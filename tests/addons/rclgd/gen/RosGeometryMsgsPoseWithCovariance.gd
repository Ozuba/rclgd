extends RosMsg
class_name RosGeometryMsgsPoseWithCovariance

const ROS_TYPE_NAME = "geometry_msgs/msg/PoseWithCovariance"

func _init():
	init(ROS_TYPE_NAME)

var pose : RosGeometryMsgsPose:
	get: return get_member(&"pose")
	set(v): set_member(&"pose", v)

var covariance : PackedFloat64Array:
	get: return get_member(&"covariance")
	set(v): set_member(&"covariance", v)

