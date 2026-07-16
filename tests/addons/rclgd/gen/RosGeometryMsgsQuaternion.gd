extends RosMsg
class_name RosGeometryMsgsQuaternion

const ROS_TYPE_NAME = "geometry_msgs/msg/Quaternion"

func _init():
	init(ROS_TYPE_NAME)

var x : float:
	get: return get_member(&"x")
	set(v): set_member(&"x", v)

var y : float:
	get: return get_member(&"y")
	set(v): set_member(&"y", v)

var z : float:
	get: return get_member(&"z")
	set(v): set_member(&"z", v)

var w : float:
	get: return get_member(&"w")
	set(v): set_member(&"w", v)

