extends "res://rclgd_test.gd"


func test_broadcast_and_lookup():
	var node := make_node("tf_loopback")
	var broadcaster := node.create_tf_broadcaster()
	var listener := node.create_tf_listener()
	var sent := Transform3D(Basis(), Vector3(1, 2, 3))

	var found := await wait_until(func():
		broadcaster.send_transform(sent, "rclgd_test_child", "rclgd_test_parent", false, node.now())
		var transform = listener.lookup_transform("rclgd_test_parent", "rclgd_test_child")
		return transform != null and transform.origin.is_equal_approx(sent.origin))
	expect(found, "transform received with the sent origin")


func test_unknown_frame_lookup_fails():
	var node := make_node("tf_unknown")
	var listener := node.create_tf_listener()
	expect_eq(listener.lookup_transform("rclgd_test_nowhere", "rclgd_test_nothing"), null, "lookup result")
	expect(not listener.get_last_error().is_empty(), "last error explains the failure")
