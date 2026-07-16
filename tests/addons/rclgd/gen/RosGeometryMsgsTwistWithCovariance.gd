extends RosMsg
class_name RosGeometryMsgsTwistWithCovariance

const ROS_TYPE_NAME = "geometry_msgs/msg/TwistWithCovariance"

func _init():
	init(ROS_TYPE_NAME)

var twist : RosGeometryMsgsTwist:
	get: return get_member(&"twist")
	set(v): set_member(&"twist", v)

var covariance : PackedFloat64Array:
	get: return get_member(&"covariance")
	set(v): set_member(&"covariance", v)

