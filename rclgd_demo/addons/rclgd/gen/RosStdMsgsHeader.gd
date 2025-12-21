extends RosMsg
class_name RosStdMsgsHeader

func _init():
	init("std_msgs/msg/Header")

var stamp : RosBuiltinInterfacesTime:
	get: return get_member(&"stamp") as RosMsg
	set(v): set_member(&"stamp", v)

var frame_id : String:
	get: return get_member(&"frame_id")
	set(v): set_member(&"frame_id", v)

