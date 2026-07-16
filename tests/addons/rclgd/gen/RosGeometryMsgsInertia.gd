extends RosMsg
class_name RosGeometryMsgsInertia

const ROS_TYPE_NAME = "geometry_msgs/msg/Inertia"

func _init():
	init(ROS_TYPE_NAME)

var m : float:
	get: return get_member(&"m")
	set(v): set_member(&"m", v)

var com : RosGeometryMsgsVector3:
	get: return get_member(&"com")
	set(v): set_member(&"com", v)

var ixx : float:
	get: return get_member(&"ixx")
	set(v): set_member(&"ixx", v)

var ixy : float:
	get: return get_member(&"ixy")
	set(v): set_member(&"ixy", v)

var ixz : float:
	get: return get_member(&"ixz")
	set(v): set_member(&"ixz", v)

var iyy : float:
	get: return get_member(&"iyy")
	set(v): set_member(&"iyy", v)

var iyz : float:
	get: return get_member(&"iyz")
	set(v): set_member(&"iyz", v)

var izz : float:
	get: return get_member(&"izz")
	set(v): set_member(&"izz", v)

