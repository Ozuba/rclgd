extends RosMsg
class_name RosGeometryMsgsVector3Stamped

const ROS_TYPE_NAME = "geometry_msgs/msg/Vector3Stamped"

func _init():
	init(ROS_TYPE_NAME)

var header : RosStdMsgsHeader:
	get: return get_member(&"header")
	set(v): set_member(&"header", v)

var vector : RosGeometryMsgsVector3:
	get: return get_member(&"vector")
	set(v): set_member(&"vector", v)

