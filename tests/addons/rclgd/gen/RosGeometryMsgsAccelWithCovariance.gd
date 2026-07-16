extends RosMsg
class_name RosGeometryMsgsAccelWithCovariance

const ROS_TYPE_NAME = "geometry_msgs/msg/AccelWithCovariance"

func _init():
	init(ROS_TYPE_NAME)

var accel : RosGeometryMsgsAccel:
	get: return get_member(&"accel")
	set(v): set_member(&"accel", v)

var covariance : PackedFloat64Array:
	get: return get_member(&"covariance")
	set(v): set_member(&"covariance", v)

