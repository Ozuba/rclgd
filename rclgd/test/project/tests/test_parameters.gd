extends "res://rclgd_test.gd"


func test_scalar_parameters():
	var node := make_node("params_scalar")

	node.declare_parameter("text", "default_value")
	expect_eq(node.get_parameter("text"), "default_value", "string default")
	node.set_parameter("text", "new_value")
	expect_eq(node.get_parameter("text"), "new_value", "string update")

	node.declare_parameter("count", 42)
	expect_eq(node.get_parameter("count"), 42, "int")

	node.declare_parameter("ratio", 3.14)
	expect_near(node.get_parameter("ratio"), 3.14, "float")

	node.declare_parameter("enabled", true)
	expect_eq(node.get_parameter("enabled"), true, "bool")


func test_parameter_changed_signal():
	var node := make_node("params_signal")
	node.declare_parameter("speed", 1.0)
	var changes := []
	node.parameter_changed.connect(func(param_name, value): changes.append([param_name, value]))
	node.set_parameter("speed", 2.5)
	expect(await wait_until(func(): return changes.has(["speed", 2.5])), "parameter_changed emitted")


func test_array_parameters():
	var node := make_node("params_array")

	for value in [
		PackedFloat64Array([1.5, 2.5, 3.5]),
		PackedInt64Array([10, 20, 30]),
		PackedStringArray(["a", "b", "c"]),
		PackedByteArray([1, 2, 3, 255]),
	]:
		var param_name := "array_%d" % typeof(value)
		node.declare_parameter(param_name, value)
		expect_eq(node.get_parameter(param_name), value, type_string(typeof(value)))

	node.set_parameter("array_%d" % TYPE_PACKED_FLOAT64_ARRAY, PackedFloat64Array([9.0]))
	expect_eq(node.get_parameter("array_%d" % TYPE_PACKED_FLOAT64_ARRAY), PackedFloat64Array([9.0]), "array update")
