extends RosMsg
class_name RosStdMsgsUInt8MultiArray

const ROS_TYPE_NAME = "std_msgs/msg/UInt8MultiArray"

func _init():
	init(ROS_TYPE_NAME)

var layout : RosStdMsgsMultiArrayLayout:
	get: return get_member(&"layout")
	set(v): set_member(&"layout", v)

var data : PackedByteArray:
	get: return get_member(&"data")
	set(v): set_member(&"data", v)

