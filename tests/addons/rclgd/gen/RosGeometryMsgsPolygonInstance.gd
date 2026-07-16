extends RosMsg
class_name RosGeometryMsgsPolygonInstance

const ROS_TYPE_NAME = "geometry_msgs/msg/PolygonInstance"

func _init():
	init(ROS_TYPE_NAME)

var polygon : RosGeometryMsgsPolygon:
	get: return get_member(&"polygon")
	set(v): set_member(&"polygon", v)

var id : int:
	get: return get_member(&"id")
	set(v): set_member(&"id", v)

