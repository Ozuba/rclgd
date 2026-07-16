extends RosMsg
class_name RosGeometryMsgsWrenchStamped

const ROS_TYPE_NAME = "geometry_msgs/msg/WrenchStamped"

func _init():
	init(ROS_TYPE_NAME)

var header : RosStdMsgsHeader:
	get: return get_member(&"header")
	set(v): set_member(&"header", v)

var wrench : RosGeometryMsgsWrench:
	get: return get_member(&"wrench")
	set(v): set_member(&"wrench", v)

