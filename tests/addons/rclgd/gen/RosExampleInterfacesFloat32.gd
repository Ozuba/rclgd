extends RosMsg
class_name RosExampleInterfacesFloat32

const ROS_TYPE_NAME = "example_interfaces/msg/Float32"

func _init():
	init(ROS_TYPE_NAME)

var data : float:
	get: return get_member(&"data")
	set(v): set_member(&"data", v)

