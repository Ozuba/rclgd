extends RosMsg
class_name RosExampleInterfacesMultiArrayDimension

const ROS_TYPE_NAME = "example_interfaces/msg/MultiArrayDimension"

func _init():
	init(ROS_TYPE_NAME)

var label : String:
	get: return get_member(&"label")
	set(v): set_member(&"label", v)

var size : int:
	get: return get_member(&"size")
	set(v): set_member(&"size", v)

var stride : int:
	get: return get_member(&"stride")
	set(v): set_member(&"stride", v)

