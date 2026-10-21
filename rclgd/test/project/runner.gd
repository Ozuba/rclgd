extends SceneTree
## Headless runner for the rclgd system tests.
##
##   godot --headless --path <project> --script res://runner.gd -- [--junit=<file>] [--filter=<text>]
##
## Runs every test_* method of every res://tests/test_*.gd script, prints one
## line per case and exits non-zero if any case failed. --filter keeps the
## cases whose "script::method" id contains the given text.

const TESTS_DIR := "res://tests"
const CASE_TIMEOUT_SEC := 30.0


## Collects the engine errors (not warnings) raised while a case runs. Errors
## can come from ROS executor threads, hence the mutex.
class ErrorCapture extends Logger:
	var _mutex := Mutex.new()
	var _errors := PackedStringArray()

	func _log_error(_function: String, file: String, line: int, code: String, rationale: String,
			_editor_notify: bool, error_type: int, _script_backtraces: Array[ScriptBacktrace]) -> void:
		if error_type == ERROR_TYPE_WARNING:
			return
		var text := ("%s: %s" % [code, rationale]) if not rationale.is_empty() else code
		_mutex.lock()
		_errors.append("%s (%s:%d)" % [text, file.get_file(), line])
		_mutex.unlock()

	func take() -> PackedStringArray:
		_mutex.lock()
		var errors := _errors
		_errors = PackedStringArray()
		_mutex.unlock()
		return errors


func _initialize() -> void:
	_run()


func _run() -> void:
	# The root only enters the tree after _initialize; suites need get_tree()
	await process_frame
	var options := _parse_options()
	var capture := ErrorCapture.new()
	OS.add_logger(capture)
	rclgd.init(PackedStringArray())

	var results: Array[Dictionary] = []
	for file in _test_files():
		var suite: Node = load(TESTS_DIR.path_join(file)).new()
		root.add_child(suite)
		for method in _test_methods(suite):
			var id := "%s::%s" % [file, method]
			if not options.filter.is_empty() and not options.filter in id:
				continue
			capture.take() # Drop anything raised between cases
			suite._failures.clear()
			suite._expected_errors.clear()

			var started := Time.get_ticks_msec()
			var finished: bool = await _run_case(suite, method)
			var failures: PackedStringArray = suite._failures.duplicate()
			if not finished:
				failures.append("timed out after %d s" % CASE_TIMEOUT_SEC)
			failures.append_array(_check_errors(capture.take(), suite._expected_errors))

			var result := {suite = file, name = method, ms = Time.get_ticks_msec() - started, failures = failures}
			results.append(result)
			_print_result(id, result)
		# Destroy the suite's ROS objects before the next suite (and shutdown)
		suite.free()

	rclgd.shutdown()
	OS.remove_logger(capture)

	var failed := results.filter(func(r): return not r.failures.is_empty()).size()
	print("\n%d passed, %d failed" % [results.size() - failed, failed])
	if not options.junit.is_empty():
		_write_junit(options.junit, results)
	quit(1 if failed > 0 or results.is_empty() else 0)


## Runs one case, racing it against CASE_TIMEOUT_SEC. Returns false on timeout.
func _run_case(suite: Node, method: String) -> bool:
	var state := {done = false}
	var body := func():
		await suite.call(method)
		state.done = true
	body.call()
	var deadline := Time.get_ticks_msec() + int(CASE_TIMEOUT_SEC * 1000.0)
	while not state.done and Time.get_ticks_msec() < deadline:
		await process_frame
	return state.done


static func _check_errors(errors: PackedStringArray, expected: PackedStringArray) -> PackedStringArray:
	var problems := PackedStringArray()
	var seen := {}
	for error in errors:
		var announced := false
		for fragment in expected:
			if fragment in error:
				seen[fragment] = true
				announced = true
		if not announced:
			problems.append("unexpected error: " + error)
	for fragment in expected:
		if not seen.has(fragment):
			problems.append("expected an error containing '%s'" % fragment)
	return problems


static func _parse_options() -> Dictionary:
	var options := {junit = "", filter = ""}
	for arg in OS.get_cmdline_user_args():
		if arg.begins_with("--junit="):
			options.junit = arg.trim_prefix("--junit=")
		elif arg.begins_with("--filter="):
			options.filter = arg.trim_prefix("--filter=")
	return options


static func _test_files() -> PackedStringArray:
	var files := Array(DirAccess.get_files_at(TESTS_DIR)).filter(
			func(f: String): return f.begins_with("test_") and f.ends_with(".gd"))
	files.sort()
	return PackedStringArray(files)


static func _test_methods(suite: Node) -> PackedStringArray:
	var methods := PackedStringArray()
	for method in suite.get_script().get_script_method_list():
		if method.name.begins_with("test_") and not methods.has(method.name):
			methods.append(method.name)
	return methods


static func _print_result(id: String, result: Dictionary) -> void:
	var status := "PASS" if result.failures.is_empty() else "FAIL"
	print("%s  %s (%d ms)" % [status, id, result.ms])
	for failure in result.failures:
		print("      - " + failure)


static func _write_junit(path: String, results: Array[Dictionary]) -> void:
	var suites := {}
	for result in results:
		if not suites.has(result.suite):
			suites[result.suite] = []
		suites[result.suite].append(result)

	var xml := PackedStringArray(['<?xml version="1.0" encoding="UTF-8"?>', "<testsuites>"])
	for suite_name in suites:
		var cases: Array = suites[suite_name]
		var failed := cases.filter(func(r): return not r.failures.is_empty()).size()
		xml.append('  <testsuite name="%s" tests="%d" failures="%d">' % [suite_name.get_basename(), cases.size(), failed])
		for result in cases:
			xml.append('    <testcase classname="%s" name="%s" time="%.3f">' % [
					suite_name.get_basename(), result.name, result.ms / 1000.0])
			if not result.failures.is_empty():
				var message := "\n".join(result.failures).xml_escape(true)
				xml.append('      <failure message="%s">%s</failure>' % [message, message])
			xml.append("    </testcase>")
		xml.append("  </testsuite>")
	xml.append("</testsuites>")

	DirAccess.make_dir_recursive_absolute(path.get_base_dir())
	var file := FileAccess.open(path, FileAccess.WRITE)
	if file == null:
		push_error("Could not write JUnit report to " + path)
		return
	file.store_string("\n".join(xml) + "\n")
