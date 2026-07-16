extends RosMsg
class_name RosExampleInterfacesFloat64

const ROS_TYPE_NAME = "example_interfaces/msg/Float64"

func _init():
	init(ROS_TYPE_NAME)

var data : float:
	get: return get_member(&"data")
	set(v): set_member(&"data", v)

