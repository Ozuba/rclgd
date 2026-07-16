extends RosMsg
class_name RosStdMsgsFloat32

const ROS_TYPE_NAME = "std_msgs/msg/Float32"

func _init():
	init(ROS_TYPE_NAME)

var data : float:
	get: return get_member(&"data")
	set(v): set_member(&"data", v)

