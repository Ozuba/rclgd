extends RosMsg
class_name RosStdMsgsMultiArrayLayout

const ROS_TYPE_NAME = "std_msgs/msg/MultiArrayLayout"

func _init():
	init(ROS_TYPE_NAME)

var dim : Array[RosStdMsgsMultiArrayDimension]:
	get: return get_member(&"dim")
	set(v): set_member(&"dim", v)

var data_offset : int:
	get: return get_member(&"data_offset")
	set(v): set_member(&"data_offset", v)

