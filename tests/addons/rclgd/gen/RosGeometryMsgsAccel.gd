extends RosMsg
class_name RosGeometryMsgsAccel

const ROS_TYPE_NAME = "geometry_msgs/msg/Accel"

func _init():
	init(ROS_TYPE_NAME)

var linear : RosGeometryMsgsVector3:
	get: return get_member(&"linear")
	set(v): set_member(&"linear", v)

var angular : RosGeometryMsgsVector3:
	get: return get_member(&"angular")
	set(v): set_member(&"angular", v)

