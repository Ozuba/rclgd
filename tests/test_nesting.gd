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
