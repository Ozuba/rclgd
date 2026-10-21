extends Node
## Base class for rclgd system tests, run by runner.gd.
##
## Every method named test_* is one test case and may await. A case fails if
## an expectation fails, if it times out, or if the engine reports an error
## while it runs, unless that error was announced with expect_error().

var _failures := PackedStringArray()
var _expected_errors := PackedStringArray()


func expect(condition: bool, what: String) -> void:
	if not condition:
		_failures.append(what)


func expect_eq(actual: Variant, expected: Variant, what: String) -> void:
	# A freed or empty object reference is TYPE_OBJECT, so null compares by value
	var equal: bool = actual == null if expected == null \
			else typeof(actual) == typeof(expected) and actual == expected
	if not equal:
		_failures.append("%s: expected %s, got %s" % [what, var_to_str(expected), var_to_str(actual)])


func expect_near(actual: float, expected: float, what: String, tolerance := 1e-6) -> void:
	if absf(actual - expected) > tolerance:
		_failures.append("%s: expected %s, got %s" % [what, expected, actual])


## Declares that the current case raises an engine error containing
## [param fragment]. The case fails if no such error is raised.
func expect_error(fragment: String) -> void:
	_expected_errors.append(fragment)


## Polls [param condition] every frame until it returns true or
## [param timeout_sec] elapses. Returns whether the condition was met.
func wait_until(condition: Callable, timeout_sec := 3.0) -> bool:
	var deadline := Time.get_ticks_msec() + int(timeout_sec * 1000.0)
	while not condition.call():
		if Time.get_ticks_msec() > deadline:
			return false
		await get_tree().process_frame
	return true


## Returns an initialized RosNode. Names only need to be unique per case.
func make_node(node_name: String) -> RosNode:
	var node := RosNode.new()
	node.init(node_name)
	return node
