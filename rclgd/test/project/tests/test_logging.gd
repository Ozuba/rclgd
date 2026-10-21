extends "res://rclgd_test.gd"


func test_node_logger_names():
	var node := make_node("logging_names")
	var logger := node.get_logger()
	expect_eq(logger.get_name(), "logging_names", "node logger name")
	expect(logger == node.get_logger(), "same instance on every call")
	expect_eq(logger.get_child("lidar").get_name(), "logging_names.lidar", "child logger name")
	expect_eq(rclgd.get_logger("rclgd_test_tool").get_name(), "rclgd_test_tool", "standalone logger name")


func test_levels():
	var logger := make_node("logging_levels").get_logger()
	var child := logger.get_child("sub")

	expect(logger.info("info is emitted by default"), "info emitted")
	expect(not logger.debug("debug is filtered by default"), "debug filtered")

	logger.set_level(RosLogger.DEBUG)
	expect_eq(logger.get_effective_level(), RosLogger.DEBUG, "explicit level")
	expect_eq(child.get_effective_level(), RosLogger.DEBUG, "child inherits level")
	expect(logger.debug("debug is emitted after set_level"), "debug emitted")

	logger.set_level(RosLogger.UNSET)
	expect_eq(logger.get_effective_level(), RosLogger.INFO, "UNSET falls back to the default")


func test_filters():
	var logger := make_node("logging_filters").get_logger()
	var once := []
	var skip_first := []
	var throttled := []
	for i in 3:
		once.append(logger.info("once", {"once": true}))
		skip_first.append(logger.info("skip_first", {"skip_first": true}))
		throttled.append(logger.info("throttled", {"throttle_duration_sec": 60.0}))
	expect_eq(once, [true, false, false], "once")
	expect_eq(skip_first, [false, true, true], "skip_first")
	expect_eq(throttled, [true, false, false], "throttle_duration_sec")


func test_invalid_filter_is_an_error():
	var logger := make_node("logging_invalid").get_logger()
	expect_error("Unsupported logging keyword argument")
	expect(not logger.info("never emitted", {"bogus": true}), "message dropped")


func test_module_functions():
	rclgd.set_logger_level("rclgd_test_module", RosLogger.ERROR)
	expect_eq(rclgd.get_logger_effective_level("rclgd_test_module"), RosLogger.ERROR, "set_logger_level")
	expect_eq(rclgd.get_logging_severity_from_string("warn"), RosLogger.WARN, "severity from string")

	expect_error("Unknown logging severity")
	expect_eq(rclgd.get_logging_severity_from_string("loud"), RosLogger.UNSET, "unknown severity")
