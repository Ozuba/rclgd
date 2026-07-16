extends RosMsg
class_name RosExampleInterfacesString

const ROS_TYPE_NAME = "example_interfaces/msg/String"

func _init():
	init(ROS_TYPE_NAME)

var data : String:
	get: return get_member(&"data")
	set(v): set_member(&"data", v)

