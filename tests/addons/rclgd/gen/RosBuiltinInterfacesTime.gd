extends RosMsg
class_name RosBuiltinInterfacesTime

const ROS_TYPE_NAME = "builtin_interfaces/msg/Time"

func _init():
	init(ROS_TYPE_NAME)

var sec : int:
	get: return get_member(&"sec")
	set(v): set_member(&"sec", v)

var nanosec : int:
	get: return get_member(&"nanosec")
	set(v): set_member(&"nanosec", v)

