extends RosMsg
class_name RosGeometryMsgsQuaternionStamped

const ROS_TYPE_NAME = "geometry_msgs/msg/QuaternionStamped"

func _init():
	init(ROS_TYPE_NAME)

var header : RosStdMsgsHeader:
	get: return get_member(&"header")
	set(v): set_member(&"header", v)

var quaternion : RosGeometryMsgsQuaternion:
	get: return get_member(&"quaternion")
	set(v): set_member(&"quaternion", v)

