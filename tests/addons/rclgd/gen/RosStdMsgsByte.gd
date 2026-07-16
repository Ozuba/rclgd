extends RosMsg
class_name RosStdMsgsByte

const ROS_TYPE_NAME = "std_msgs/msg/Byte"

func _init():
	init(ROS_TYPE_NAME)

var data : int:
	get: return get_member(&"data")
	set(v): set_member(&"data", v)

