extends RosMsg
class_name RosExampleInterfacesByteMultiArray

const ROS_TYPE_NAME = "example_interfaces/msg/ByteMultiArray"

func _init():
	init(ROS_TYPE_NAME)

var layout : RosExampleInterfacesMultiArrayLayout:
	get: return get_member(&"layout")
	set(v): set_member(&"layout", v)

var data : PackedByteArray:
	get: return get_member(&"data")
	set(v): set_member(&"data", v)

