extends RosMsg
class_name RosStdMsgsString

func _init():
	init("std_msgs/msg/String")

var data : String:
	get: return get_member(&"data")
	set(v): set_member(&"data", v)

