extends RosMsg
class_name RosExampleInterfacesInt64MultiArray

const ROS_TYPE_NAME = "example_interfaces/msg/Int64MultiArray"

func _init():
	init(ROS_TYPE_NAME)

var layout : RosExampleInterfacesMultiArrayLayout:
	get: return get_member(&"layout")
	set(v): set_member(&"layout", v)

var data : PackedInt64Array:
	get: return get_member(&"data")
	set(v): set_member(&"data", v)

