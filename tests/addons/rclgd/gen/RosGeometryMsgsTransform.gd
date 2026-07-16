extends RosMsg
class_name RosGeometryMsgsTransform

const ROS_TYPE_NAME = "geometry_msgs/msg/Transform"

func _init():
	init(ROS_TYPE_NAME)

var translation : RosGeometryMsgsVector3:
	get: return get_member(&"translation")
	set(v): set_member(&"translation", v)

var rotation : RosGeometryMsgsQuaternion:
	get: return get_member(&"rotation")
	set(v): set_member(&"rotation", v)

