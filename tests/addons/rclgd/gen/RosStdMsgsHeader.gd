extends RosMsg
class_name RosStdMsgsHeader

const ROS_TYPE_NAME = "std_msgs/msg/Header"

func _init():
	init(ROS_TYPE_NAME)

var stamp : RosBuiltinInterfacesTime:
	get: return get_member(&"stamp")
	set(v): set_member(&"stamp", v)

var frame_id : String:
	get: return get_member(&"frame_id")
	set(v): set_member(&"frame_id", v)

