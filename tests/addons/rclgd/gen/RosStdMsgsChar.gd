extends RosMsg
class_name RosStdMsgsChar

const ROS_TYPE_NAME = "std_msgs/msg/Char"

func _init():
	init(ROS_TYPE_NAME)

var data : int:
	get: return get_member(&"data")
	set(v): set_member(&"data", v)

