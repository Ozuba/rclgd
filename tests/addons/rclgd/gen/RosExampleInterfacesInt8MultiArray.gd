extends RosMsg
class_name RosExampleInterfacesInt8MultiArray

const ROS_TYPE_NAME = "example_interfaces/msg/Int8MultiArray"

func _init():
	init(ROS_TYPE_NAME)

var layout : RosExampleInterfacesMultiArrayLayout:
	get: return get_member(&"layout")
	set(v): set_member(&"layout", v)

var data : PackedInt32Array:
	get: return get_member(&"data")
	set(v): set_member(&"data", v)

