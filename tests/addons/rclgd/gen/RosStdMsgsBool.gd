extends RosMsg
class_name RosStdMsgsBool

const ROS_TYPE_NAME = "std_msgs/msg/Bool"

func _init():
	init(ROS_TYPE_NAME)

var data : bool:
	get: return get_member(&"data")
	set(v): set_member(&"data", v)

