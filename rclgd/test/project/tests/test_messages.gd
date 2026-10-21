extends "res://rclgd_test.gd"


func test_nested_assignment():
	var msg := RosMsg.from_type("geometry_msgs/msg/PoseStamped")
	msg.header.frame_id = "base_link"
	msg.header.stamp.sec = 5
	msg.pose.position.x = 1.5
	msg.pose.orientation.w = 1.0

	expect_eq(msg.header.frame_id, "base_link", "header.frame_id")
	expect_eq(msg.header.stamp.sec, 5, "header.stamp.sec")
	expect_near(msg.pose.position.x, 1.5, "pose.position.x")
	expect_near(msg.pose.orientation.w, 1.0, "pose.orientation.w")


func test_type_names():
	var msg := RosMsg.from_type("geometry_msgs/msg/PoseStamped")
	expect_eq(msg.get_ros_interface_name(), "geometry_msgs/msg/PoseStamped", "interface name")


func test_unknown_type_returns_null():
	expect_error("does_not_exist")
	expect_eq(RosMsg.from_type("does_not_exist/msg/Nope"), null, "from_type result")
