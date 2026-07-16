extends RosMsg
class_name RosGeometryMsgsAccelStamped

const ROS_TYPE_NAME = "geometry_msgs/msg/AccelStamped"

func _init():
	init(ROS_TYPE_NAME)

var header : RosStdMsgsHeader:
	get: return get_member(&"header")
	set(v): set_member(&"header", v)

var accel : RosGeometryMsgsAccel:
	get: return get_member(&"accel")
	set(v): set_member(&"accel", v)

