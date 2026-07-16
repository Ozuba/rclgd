extends RosMsg
class_name RosGeometryMsgsPoseArray

const ROS_TYPE_NAME = "geometry_msgs/msg/PoseArray"

func _init():
	init(ROS_TYPE_NAME)

var header : RosStdMsgsHeader:
	get: return get_member(&"header")
	set(v): set_member(&"header", v)

var poses : Array[RosGeometryMsgsPose]:
	get: return get_member(&"poses")
	set(v): set_member(&"poses", v)

