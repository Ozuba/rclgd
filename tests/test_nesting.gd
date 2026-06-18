extends Test

func _execute():
	var msg = RosMsg.from_type("geometry_msgs/msg/PoseStamped")
	
	# Deep assignment
	msg.header.frame_id = "base_link"
	msg.pose.position.x = 1.5
	msg.pose.orientation.w = 1.0
	
	# Verify that values persist
	assert_equal(msg.header.frame_id, "base_link", "Frame ID")
	assert_equal(abs(msg.pose.position.x - 1.5) < 0.001, true, "Position X")
	assert_equal(abs(msg.pose.orientation.w - 1.0) < 0.001, true, "Orientation W")

	# --- Typed (shadow) path ---
	# Construct via the generated shadow class and assign through nested typed
	# properties. Regression guard for the Godot 4.7 break where nested getters
	# returned a bare RosMsg, got coerced to null against the typed property, and
	# any chained write (msg.header.stamp.sec = x) failed on a Nil base object.
	var typed := RosGeometryMsgsPoseStamped.new()
	typed.header.frame_id = "map"
	typed.header.stamp.sec = 5
	typed.header.stamp.nanosec = 123

	# Nested getters must return the typed instances, not Nil.
	assert_equal(typed.header is RosStdMsgsHeader, true, "header is RosStdMsgsHeader")
	assert_equal(typed.header.stamp is RosBuiltinInterfacesTime, true, "stamp is RosBuiltinInterfacesTime")

	# Values must round-trip through the ROS buffer.
	assert_equal(typed.header.frame_id, "map", "Typed Frame ID")
	assert_equal(typed.header.stamp.sec, 5, "Typed stamp.sec")
	assert_equal(typed.header.stamp.nanosec, 123, "Typed stamp.nanosec")

	# The factory must also hand back a typed instance.
	var made := RosMsg.from_type("geometry_msgs/msg/PoseStamped")
	assert_equal(made is RosGeometryMsgsPoseStamped, true, "from_type returns typed instance")
