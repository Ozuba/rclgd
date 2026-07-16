extends RosMsg
class_name RosGeometryMsgsTwistWithCovarianceStamped

const ROS_TYPE_NAME = "geometry_msgs/msg/TwistWithCovarianceStamped"

func _init():
	init(ROS_TYPE_NAME)

var header : RosStdMsgsHeader:
	get: return get_member(&"header")
	set(v): set_member(&"header", v)

var twist : RosGeometryMsgsTwistWithCovariance:
	get: return get_member(&"twist")
	set(v): set_member(&"twist", v)

