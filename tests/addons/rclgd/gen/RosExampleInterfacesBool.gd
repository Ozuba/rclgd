extends RosMsg
class_name RosExampleInterfacesBool

const ROS_TYPE_NAME = "example_interfaces/msg/Bool"

func _init():
	init(ROS_TYPE_NAME)

var data : bool:
	get: return get_member(&"data")
	set(v): set_member(&"data", v)

