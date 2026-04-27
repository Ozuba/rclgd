# ros_test_base.gd
class_name Test
extends Node

# Signal for the runner to know we are done
signal test_finished(success: bool)

# Internal tracking
var _failed := false

# Helper: Custom assertion
func assert_equal(actual, expected, msg: String = ""):
	if actual != expected:
		printerr("  [FAIL] %s: Expected %s, got %s" % [msg, str(expected), str(actual)])
		_failed = true

# Virtual methods to be overridden
func before_test(): pass
func after_test(): pass

# The main execution loop
func run_test() -> bool:
	await before_test()
	await _execute() # This is where the actual test logic goes
	await after_test()
	return not _failed

# To be implemented by children
func _execute(): pass
