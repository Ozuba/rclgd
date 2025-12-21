extends RosMsg
class_name RosSensorMsgsPointCloud2

func _init():
	init("sensor_msgs/msg/PointCloud2")

var header : RosStdMsgsHeader:
	get: return get_member(&"header") as RosMsg
	set(v): set_member(&"header", v)

var height : int:
	get: return get_member(&"height")
	set(v): set_member(&"height", v)

var width : int:
	get: return get_member(&"width")
	set(v): set_member(&"width", v)

var fields : Array:
	get: return get_member(&"fields")
	set(v): set_member(&"fields", v)

var is_bigendian : int:
	get: return get_member(&"is_bigendian")
	set(v): set_member(&"is_bigendian", v)

var point_step : int:
	get: return get_member(&"point_step")
	set(v): set_member(&"point_step", v)

var row_step : int:
	get: return get_member(&"row_step")
	set(v): set_member(&"row_step", v)

var data : PackedByteArray:
	get: return get_member(&"data")
	set(v): set_member(&"data", v)

var is_dense : int:
	get: return get_member(&"is_dense")
	set(v): set_member(&"is_dense", v)

