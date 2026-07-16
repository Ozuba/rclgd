extends RosMsg
class_name RosGeometryMsgsPose

const ROS_TYPE_NAME = "geometry_msgs/msg/Pose"

func _init():
	init(ROS_TYPE_NAME)

var position : RosGeometryMsgsPoint:
	get: return get_member(&"position")
	set(v): set_member(&"position", v)

var orientation : RosGeometryMsgsQuaternion:
	get: return get_member(&"orientation")
	set(v): set_member(&"orientation", v)

