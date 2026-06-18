extends Test

func _execute():
	var node = RosNode.new()
	node.init("test_params_node")
	
	# Declare and Get Parameter
	node.declare_parameter("test_string", "default_value")
	var val1 = node.get_parameter("test_string")
	assert_equal(val1, "default_value", "String parameter default")
		
	# Set Parameter
	node.set_parameter("test_string", "new_value")
	var val2 = node.get_parameter("test_string")
	assert_equal(val2, "new_value", "String parameter update")
		
	# Test different types
	node.declare_parameter("test_int", 42)
	var val_int = node.get_parameter("test_int")
	assert_equal(val_int, 42, "Integer parameter")
		
	node.declare_parameter("test_float", 3.14)
	var val_float = node.get_parameter("test_float")
	assert_equal(abs(val_float - 3.14) < 0.001, true, "Float parameter")
		
	node.declare_parameter("test_bool", true)
	var val_bool = node.get_parameter("test_bool")
	assert_equal(val_bool, true, "Boolean parameter")

	# Array parameter types (round-trip through the ROS parameter server)
	var floats := PackedFloat64Array([1.5, 2.5, 3.5])
	node.declare_parameter("test_floats", floats)
	assert_equal(node.get_parameter("test_floats"), floats, "Float64 array parameter")

	var ints := PackedInt64Array([10, 20, 30])
	node.declare_parameter("test_ints", ints)
	assert_equal(node.get_parameter("test_ints"), ints, "Int64 array parameter")

	var strs := PackedStringArray(["a", "b", "c"])
	node.declare_parameter("test_strs", strs)
	assert_equal(node.get_parameter("test_strs"), strs, "String array parameter")

	var bytes := PackedByteArray([1, 2, 3, 255])
	node.declare_parameter("test_bytes", bytes)
	assert_equal(node.get_parameter("test_bytes"), bytes, "Byte array parameter")

	# Update an array parameter after declaration
	node.set_parameter("test_floats", PackedFloat64Array([9.0]))
	assert_equal(node.get_parameter("test_floats"), PackedFloat64Array([9.0]), "Float64 array update")
