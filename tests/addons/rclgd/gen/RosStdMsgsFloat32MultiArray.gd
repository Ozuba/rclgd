extends RosMsg
class_name RosStdMsgsFloat32MultiArray

const ROS_TYPE_NAME = "std_msgs/msg/Float32MultiArray"

func _init():
	init(ROS_TYPE_NAME)

var layout : RosStdMsgsMultiArrayLayout:
	get: return get_member(&"layout")
	set(v): set_member(&"layout", v)

var data : PackedFloat32Array:
	get: return get_member(&"data")
	set(v): set_member(&"data", v)

