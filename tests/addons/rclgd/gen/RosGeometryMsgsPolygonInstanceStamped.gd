extends RosMsg
class_name RosGeometryMsgsPolygonInstanceStamped

const ROS_TYPE_NAME = "geometry_msgs/msg/PolygonInstanceStamped"

func _init():
	init(ROS_TYPE_NAME)

var header : RosStdMsgsHeader:
	get: return get_member(&"header")
	set(v): set_member(&"header", v)

var polygon : RosGeometryMsgsPolygonInstance:
	get: return get_member(&"polygon")
	set(v): set_member(&"polygon", v)

