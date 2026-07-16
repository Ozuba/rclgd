extends RosMsg
class_name RosGeometryMsgsPolygonStamped

const ROS_TYPE_NAME = "geometry_msgs/msg/PolygonStamped"

func _init():
	init(ROS_TYPE_NAME)

var header : RosStdMsgsHeader:
	get: return get_member(&"header")
	set(v): set_member(&"header", v)

var polygon : RosGeometryMsgsPolygon:
	get: return get_member(&"polygon")
	set(v): set_member(&"polygon", v)

