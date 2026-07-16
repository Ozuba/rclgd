extends RosMsg
class_name RosGeometryMsgsWrench

const ROS_TYPE_NAME = "geometry_msgs/msg/Wrench"

func _init():
	init(ROS_TYPE_NAME)

var force : RosGeometryMsgsVector3:
	get: return get_member(&"force")
	set(v): set_member(&"force", v)

var torque : RosGeometryMsgsVector3:
	get: return get_member(&"torque")
	set(v): set_member(&"torque", v)

