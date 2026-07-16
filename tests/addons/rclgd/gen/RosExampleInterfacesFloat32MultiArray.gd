extends RosMsg
class_name RosExampleInterfacesFloat32MultiArray

const ROS_TYPE_NAME = "example_interfaces/msg/Float32MultiArray"

func _init():
	init(ROS_TYPE_NAME)

var layout : RosExampleInterfacesMultiArrayLayout:
	get: return get_member(&"layout")
	set(v): set_member(&"layout", v)

var data : PackedFloat32Array:
	get: return get_member(&"data")
	set(v): set_member(&"data", v)

