extends RosMsg
class_name RosGeometryMsgsPose2d

const ROS_TYPE_NAME = "geometry_msgs/msg/Pose2D"

func _init():
	init(ROS_TYPE_NAME)

var x : float:
	get: return get_member(&"x")
	set(v): set_member(&"x", v)

var y : float:
	get: return get_member(&"y")
	set(v): set_member(&"y", v)

var theta : float:
	get: return get_member(&"theta")
	set(v): set_member(&"theta", v)

