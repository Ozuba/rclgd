extends RosMsg
class_name RosGeometryMsgsPolygon

const ROS_TYPE_NAME = "geometry_msgs/msg/Polygon"

func _init():
	init(ROS_TYPE_NAME)

var points : Array[RosGeometryMsgsPoint32]:
	get: return get_member(&"points")
	set(v): set_member(&"points", v)

