extends RosMsg
class_name RosGeometryMsgsPointStamped

const ROS_TYPE_NAME = "geometry_msgs/msg/PointStamped"

func _init():
	init(ROS_TYPE_NAME)

var header : RosStdMsgsHeader:
	get: return get_member(&"header")
	set(v): set_member(&"header", v)

var point : RosGeometryMsgsPoint:
	get: return get_member(&"point")
	set(v): set_member(&"point", v)

