extends RosMsg
class_name RosGeometryMsgsVelocityWithCovarianceStamped

const ROS_TYPE_NAME = "geometry_msgs/msg/VelocityWithCovarianceStamped"

func _init():
	init(ROS_TYPE_NAME)

var header : RosStdMsgsHeader:
	get: return get_member(&"header")
	set(v): set_member(&"header", v)

var body_frame_id : String:
	get: return get_member(&"body_frame_id")
	set(v): set_member(&"body_frame_id", v)

var reference_frame_id : String:
	get: return get_member(&"reference_frame_id")
	set(v): set_member(&"reference_frame_id", v)

var velocity : RosGeometryMsgsTwistWithCovariance:
	get: return get_member(&"velocity")
	set(v): set_member(&"velocity", v)

