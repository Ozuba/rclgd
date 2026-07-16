extends RosMsg
class_name RosStdMsgsColorRgba

const ROS_TYPE_NAME = "std_msgs/msg/ColorRGBA"

func _init():
	init(ROS_TYPE_NAME)

var r : float:
	get: return get_member(&"r")
	set(v): set_member(&"r", v)

var g : float:
	get: return get_member(&"g")
	set(v): set_member(&"g", v)

var b : float:
	get: return get_member(&"b")
	set(v): set_member(&"b", v)

var a : float:
	get: return get_member(&"a")
	set(v): set_member(&"a", v)

