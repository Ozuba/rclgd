extends "res://rclgd_test.gd"
## Native typed message classes registered in the ClassDB (RosGeometryMsgsTwist, ...).


func test_new_and_nested_typed():
	var t := RosGeometryMsgsTwist.new()
	t.linear.x = 1.0
	expect(t.linear is RosGeometryMsgsVector3, "t.linear is RosGeometryMsgsVector3")
	# The nested wrapper aliases the parent's buffer, so the write sticks
	expect_near(t.linear.x, 1.0, "linear.x after nested write")
	expect_eq(t.get_ros_interface_name(), "geometry_msgs/msg/Twist", "interface name")
	expect_eq(t.get_class(), "RosGeometryMsgsTwist", "get_class")
	expect_eq(t.get_type_name(), "RosGeometryMsgsTwist", "get_type_name")


func test_typed_is_rosmsg():
	var t := RosGeometryMsgsTwist.new()
	expect(t is RosMsg, "t is RosMsg")
	var as_base: RosMsg = t
	expect_near(as_base.get_member(&"linear").x, 0.0, "get_member through the base type")


func test_from_type_returns_typed():
	var msg := RosMsg.from_type("geometry_msgs/msg/Twist")
	expect(msg is RosGeometryMsgsTwist, "from_type returns the native class")


func test_assign_nested_copies_into_parent():
	var t := RosGeometryMsgsTwist.new()
	var v := RosGeometryMsgsVector3.new()
	v.z = 2.5
	t.angular = v
	expect_near(t.angular.z, 2.5, "angular.z after assignment")
	# Assignment copies; later writes go to t's own buffer, not to v
	t.angular.z = 4.0
	expect_near(t.angular.z, 4.0, "angular.z after write through the parent")
	expect_near(v.z, 2.5, "source vector unchanged")


func test_property_list_has_no_duplicates():
	var names := (RosGeometryMsgsTwist.new().get_property_list() as Array) \
			.map(func(p): return p.name)
	expect_eq(names.count("linear"), 1, "linear listed once")
	expect_eq(names.count("angular"), 1, "angular listed once")


func test_to_string():
	var t := RosGeometryMsgsTwist.new()
	expect(str(t).begins_with("RosGeometryMsgsTwist {"), "str() of a typed message")


func test_unregistered_type_stays_dynamic():
	# rosgraph_msgs is not among res://package.xml's dependencies
	var msg := RosMsg.from_type("rosgraph_msgs/msg/Clock")
	expect_eq(msg.get_class(), "RosMsg", "unregistered type is a plain RosMsg")
	msg.clock.sec = 7
	expect_eq(msg.clock.sec, 7, "dynamic field access")


func test_dynamic_rosmsg_of_registered_type():
	# RosMsg.new() + init() stays dynamic even for a registered type
	var msg := RosMsg.new()
	msg.init("std_msgs/msg/String")
	expect_eq(msg.get_class(), "RosMsg", "still a plain RosMsg")
	msg.data = "dynamic"
	expect_eq(msg.data, "dynamic", "dynamic field access")


func test_typed_publish_and_receive():
	var node := make_node("native_twist")
	var received := []
	var _sub := node.create_subscription("/rclgd_test/twist", "geometry_msgs/msg/Twist",
			func(msg): received.append(msg))
	var pub := node.create_publisher("/rclgd_test/twist", "geometry_msgs/msg/Twist")

	var t := RosGeometryMsgsTwist.new()
	t.linear.x = 1.0
	t.angular.z = -0.5
	expect(await wait_until(func():
		pub.publish(t)
		return not received.is_empty()), "message delivered")
	if not received.is_empty():
		var got = received[0]
		expect(got is RosGeometryMsgsTwist, "received message is RosGeometryMsgsTwist")
		expect(got.linear is RosGeometryMsgsVector3, "received nested is RosGeometryMsgsVector3")
		expect_near(got.linear.x, 1.0, "linear.x")
		expect_near(got.angular.z, -0.5, "angular.z")


func test_transitive_types_registered():
	# Header is nested in geometry_msgs, Time in Header: registered by closure
	var h := RosStdMsgsHeader.new()
	h.stamp.sec = 3
	expect(h.stamp is RosBuiltinInterfacesTime, "header.stamp is RosBuiltinInterfacesTime")
	expect_eq(h.stamp.sec, 3, "stamp.sec")
	expect(make_node("native_now").now() is RosBuiltinInterfacesTime, "RosNode.now() is typed")


func test_typed_message_array():
	var pa := RosGeometryMsgsPoseArray.new()
	expect(pa.poses.is_typed(), "poses is a typed array")
	expect_eq(pa.poses.get_typed_class_name(), &"RosGeometryMsgsPose", "element class")
	var pose := RosGeometryMsgsPose.new()
	pose.position.x = 2.0
	pa.poses = [pose] as Array[RosGeometryMsgsPose]
	var poses: Array[RosGeometryMsgsPose] = pa.poses
	expect_eq(poses.size(), 1, "poses size")
	expect_near(poses[0].position.x, 2.0, "poses[0].position.x")


func test_typed_message_array_loopback():
	var node := make_node("native_pose_array")
	var received := []
	var _sub := node.create_subscription("/rclgd_test/poses", "geometry_msgs/msg/PoseArray",
			func(msg): received.append(msg))
	var pub := node.create_publisher("/rclgd_test/poses", "geometry_msgs/msg/PoseArray")

	var pa := RosGeometryMsgsPoseArray.new()
	pa.header.frame_id = "map"
	var pose := RosGeometryMsgsPose.new()
	pose.orientation.w = 1.0
	pa.poses = [pose, pose] as Array[RosGeometryMsgsPose]
	expect(await wait_until(func():
		pub.publish(pa)
		return not received.is_empty()), "message delivered")
	if not received.is_empty():
		var got: RosGeometryMsgsPoseArray = received[0]
		expect_eq(got.header.frame_id, "map", "header.frame_id")
		expect_eq(got.poses.size(), 2, "poses size")
		expect(got.poses[1] is RosGeometryMsgsPose, "received element is RosGeometryMsgsPose")
		expect_near(got.poses[1].orientation.w, 1.0, "poses[1].orientation.w")


## Whether [param source] compiles as a GDScript.
func _compiles(source: String) -> bool:
	var script := GDScript.new()
	script.source_code = source
	return script.reload() == OK


func test_properties_are_statically_typed():
	# GDScript types a native property from its getter's return type, so this
	# guards the per-field typed accessors (a shared Variant getter would leave
	# every field untyped).
	expect(_compiles("func f(t: RosGeometryMsgsTwist) -> float:\n\tvar v := t.linear\n\treturn v.x\n"),
			"field type is inferred")
	expect_error("RosGeometryMsgsVector3")
	expect(not _compiles("func f(t: RosGeometryMsgsTwist) -> String:\n\tvar s: String = t.linear\n\treturn s\n"),
			"assigning a Vector3 message to a String is a parse error")


func test_ptrcall_accessors():
	# Statically typed calls go through the accessors' ptrcall path
	var s := RosStdMsgsString.new()
	s._ros_set_data("hi")
	expect_eq(s._ros_get_data(), "hi", "String")
	var b := RosStdMsgsBool.new()
	b._ros_set_data(true)
	expect_eq(b._ros_get_data(), true, "bool")
	var i := RosStdMsgsInt32.new()
	i._ros_set_data(-42)
	expect_eq(i._ros_get_data(), -42, "int")
	var f := RosStdMsgsFloat64MultiArray.new()
	f._ros_set_data(PackedFloat64Array([1.5, 2.5]))
	expect_eq(f._ros_get_data(), PackedFloat64Array([1.5, 2.5]), "PackedFloat64Array")
	var t := RosGeometryMsgsTwist.new()
	var v := RosGeometryMsgsVector3.new()
	v.y = 6.0
	t._ros_set_linear(v)
	var linear: RosGeometryMsgsVector3 = t._ros_get_linear()
	expect_near(linear.y, 6.0, "Object")
	var pa := RosGeometryMsgsPoseArray.new()
	var poses: Array[RosGeometryMsgsPose] = pa._ros_get_poses()
	expect_eq(poses.size(), 0, "typed Array")


func test_create_entities_from_class_or_instance():
	var node := make_node("native_type_args")
	var pub := node.create_publisher("/rclgd_test/by_class", RosGeometryMsgsTwist)
	expect(pub != null, "publisher from a native class")
	var pub2 := node.create_publisher("/rclgd_test/by_instance", RosStdMsgsString.new())
	expect(pub2 != null, "publisher from a message instance")


func test_typed_service():
	var node := make_node("native_service")
	var seen := {}
	var _srv := node.create_service("/rclgd_test/native_add", "example_interfaces/srv/AddTwoInts",
			func(req, res):
				seen.req = req is RosExampleInterfacesSrvAddTwoIntsRequest
				seen.res = res is RosExampleInterfacesSrvAddTwoIntsResponse
				res.sum = req.a + req.b)
	var client := node.create_client("/rclgd_test/native_add", "example_interfaces/srv/AddTwoInts")
	expect(client.wait_for_service(3.0), "service available")

	var req := client.create_request()
	expect(req is RosExampleInterfacesSrvAddTwoIntsRequest, "create_request() is typed")
	req.a = 4
	req.b = 5
	var res: RosMsg = await client.async_send_request(req).completed
	expect(res is RosExampleInterfacesSrvAddTwoIntsResponse, "response is typed")
	expect_eq(seen.get("req"), true, "server request is typed")
	expect_eq(seen.get("res"), true, "server response is typed")
	if res != null:
		expect_eq(res.sum, 9, "sum")


func test_typed_action():
	var node := make_node("native_action")
	var seen := {}
	var _server := node.create_action_server("/rclgd_test/native_fibonacci", "example_interfaces/action/Fibonacci",
			func(gh: RosServerGoalHandle):
				seen.goal = gh.get_goal() is RosExampleInterfacesActionFibonacciGoal
				var fb := gh.create_feedback()
				seen.feedback = fb is RosExampleInterfacesActionFibonacciFeedback
				fb.sequence = PackedInt32Array([0, 1])
				gh.publish_feedback(fb)
				await get_tree().create_timer(0.05).timeout
				var result := gh.create_result()
				seen.result = result is RosExampleInterfacesActionFibonacciResult
				result.sequence = PackedInt32Array([0, 1, 1])
				gh.succeed(result))
	var client := node.create_action_client("/rclgd_test/native_fibonacci", "example_interfaces/action/Fibonacci")
	expect(client.wait_for_server(3.0), "server available")

	var goal := client.create_goal()
	expect(goal is RosExampleInterfacesActionFibonacciGoal, "create_goal() is typed")
	goal.order = 3
	var goal_handle := client.send_goal(goal)
	var feedback := []
	goal_handle.feedback.connect(func(msg): feedback.append(msg))
	await goal_handle.completed

	expect_eq(seen.get("goal"), true, "server goal is typed")
	expect_eq(seen.get("feedback"), true, "create_feedback() is typed")
	expect_eq(seen.get("result"), true, "create_result() is typed")
	expect(not feedback.is_empty() and feedback[0] is RosExampleInterfacesActionFibonacciFeedback, "client feedback is typed")
	expect(goal_handle.get_result() is RosExampleInterfacesActionFibonacciResult, "client result is typed")
	if goal_handle.get_result() != null:
		expect_eq(goal_handle.get_result().sequence, PackedInt32Array([0, 1, 1]), "result.sequence")


class ScaledTwist extends RosGeometryMsgsTwist:
	func scaled(k: float) -> float:
		return linear.x * k


func test_script_extending_typed_class():
	var t := ScaledTwist.new()
	t.linear.x = 2.0
	expect(t is RosGeometryMsgsTwist, "script instance is RosGeometryMsgsTwist")
	expect_near(t.scaled(3.0), 6.0, "script method reads typed fields")
	expect_eq(t.get_ros_interface_name(), "geometry_msgs/msg/Twist", "interface name")
