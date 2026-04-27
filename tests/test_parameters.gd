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
