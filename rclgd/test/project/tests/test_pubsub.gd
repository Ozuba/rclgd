extends "res://rclgd_test.gd"


## Publishes [param msg] until the subscription callback has filled
## [param received]: discovery between two endpoints is asynchronous, so a
## single publish right after creation may be lost.
func _publish_until_received(pub: RosPublisher, msg: RosMsg, received: Array) -> bool:
	return await wait_until(func():
		pub.publish(msg)
		return not received.is_empty())


func test_string_loopback():
	var node := make_node("pubsub_string")
	var received := []
	var _sub := node.create_subscription("/rclgd_test/string", "std_msgs/msg/String",
			func(msg): received.append(msg.data))
	var pub := node.create_publisher("/rclgd_test/string", "std_msgs/msg/String")

	var msg := RosMsg.from_type("std_msgs/msg/String")
	msg.data = "hey_from_godot"
	expect(await _publish_until_received(pub, msg, received), "message delivered")
	if not received.is_empty():
		expect_eq(received[0], "hey_from_godot", "payload")


func test_nested_message_loopback():
	var node := make_node("pubsub_nested")
	var received := []
	var _sub := node.create_subscription("/rclgd_test/pose", "geometry_msgs/msg/PoseStamped",
			func(msg): received.append(msg))
	var pub := node.create_publisher("/rclgd_test/pose", "geometry_msgs/msg/PoseStamped")

	var msg := RosMsg.from_type("geometry_msgs/msg/PoseStamped")
	msg.header.frame_id = "base_link"
	msg.pose.position.x = 1.5
	msg.pose.orientation.w = 1.0
	expect(await _publish_until_received(pub, msg, received), "message delivered")
	if not received.is_empty():
		expect_eq(received[0].header.frame_id, "base_link", "header.frame_id")
		expect_near(received[0].pose.position.x, 1.5, "pose.position.x")
		expect_near(received[0].pose.orientation.w, 1.0, "pose.orientation.w")


func test_array_loopback():
	var node := make_node("pubsub_array")
	var received := []
	var _sub := node.create_subscription("/rclgd_test/floats", "std_msgs/msg/Float64MultiArray",
			func(msg): received.append(msg.data))
	var pub := node.create_publisher("/rclgd_test/floats", "std_msgs/msg/Float64MultiArray")

	var msg := RosMsg.from_type("std_msgs/msg/Float64MultiArray")
	msg.data = PackedFloat64Array([1.5, -2.0, 3.25])
	expect(await _publish_until_received(pub, msg, received), "message delivered")
	if not received.is_empty():
		expect_eq(received[0], PackedFloat64Array([1.5, -2.0, 3.25]), "data")


func test_graph_counts():
	var node := make_node("pubsub_graph")
	var _pub := node.create_publisher("/rclgd_test/graph", "std_msgs/msg/String")
	var _sub := node.create_subscription("/rclgd_test/graph", "std_msgs/msg/String", func(_msg): pass)
	expect(await wait_until(func(): return node.count_publishers("/rclgd_test/graph") == 1), "one publisher")
	expect(await wait_until(func(): return node.count_subscribers("/rclgd_test/graph") == 1), "one subscriber")
