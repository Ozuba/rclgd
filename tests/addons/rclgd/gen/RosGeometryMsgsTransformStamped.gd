extends RosMsg
class_name RosGeometryMsgsTransformStamped

const ROS_TYPE_NAME = "geometry_msgs/msg/TransformStamped"

func _init():
	init(ROS_TYPE_NAME)

var header : RosStdMsgsHeader:
	get: return get_member(&"header")
	set(v): set_member(&"header", v)

var child_frame_id : String:
	get: return get_member(&"child_frame_id")
	set(v): set_member(&"child_frame_id", v)

var transform : RosGeometryMsgsTransform:
	get: return get_member(&"transform")
	set(v): set_member(&"transform", v)

