extends RosMsg
class_name RosExampleInterfacesInt64

const ROS_TYPE_NAME = "example_interfaces/msg/Int64"

func _init():
	init(ROS_TYPE_NAME)

var data : int:
	get: return get_member(&"data")
	set(v): set_member(&"data", v)

