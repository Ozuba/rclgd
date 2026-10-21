extends "res://rclgd_test.gd"

const FIBONACCI := "example_interfaces/action/Fibonacci"


## Computes the Fibonacci sequence up to goal.order, one step per frame with
## feedback, and honours cancel requests.
func _execute_fibonacci(goal_handle: RosServerGoalHandle) -> void:
	var sequence := [0, 1]
	for i in range(2, goal_handle.get_goal().order):
		if goal_handle.is_cancel_requested():
			var canceled := goal_handle.create_result()
			canceled.sequence = sequence
			goal_handle.canceled(canceled)
			return
		sequence.append(sequence[i - 1] + sequence[i - 2])
		var feedback := goal_handle.create_feedback()
		feedback.sequence = sequence
		goal_handle.publish_feedback(feedback)
		await get_tree().create_timer(0.01).timeout
	var result := goal_handle.create_result()
	result.sequence = sequence
	goal_handle.succeed(result)


func _send(client: RosActionClient, order: int) -> RosGoalHandle:
	var goal := client.create_goal()
	goal.order = order
	return client.send_goal(goal)


func test_goal_succeeds_with_feedback():
	var node := make_node("actions_success")
	var _server := node.create_action_server("/rclgd_test/fibonacci", FIBONACCI, _execute_fibonacci)
	var client := node.create_action_client("/rclgd_test/fibonacci", FIBONACCI)
	expect(client.wait_for_server(3.0), "server available")

	var goal_handle := _send(client, 8)
	var feedback := []
	goal_handle.feedback.connect(func(msg): feedback.append(msg))
	await goal_handle.completed

	expect(goal_handle.is_accepted(), "goal accepted")
	expect_eq(goal_handle.get_status(), RosGoalHandle.STATUS_SUCCEEDED, "status")
	expect(not feedback.is_empty(), "feedback received")
	var result := goal_handle.get_result()
	if result != null:
		expect_eq(result.sequence.size(), 8, "sequence length")
		expect_eq(int(result.sequence[7]), 13, "8th Fibonacci number")


func test_goal_callback_rejects():
	var node := make_node("actions_reject")
	var _server := node.create_action_server("/rclgd_test/fibonacci_limited", FIBONACCI, _execute_fibonacci,
			func(goal): return goal.order <= 10)
	var client := node.create_action_client("/rclgd_test/fibonacci_limited", FIBONACCI)
	expect(client.wait_for_server(3.0), "server available")

	var goal_handle := _send(client, 50)
	await goal_handle.completed
	expect(not goal_handle.is_accepted(), "goal rejected")
	expect_eq(goal_handle.get_result(), null, "no result for a rejected goal")


func test_goal_can_be_aborted():
	var node := make_node("actions_abort")
	var _server := node.create_action_server("/rclgd_test/fibonacci_abort", FIBONACCI,
			func(goal_handle): goal_handle.abort(goal_handle.create_result()))
	var client := node.create_action_client("/rclgd_test/fibonacci_abort", FIBONACCI)
	expect(client.wait_for_server(3.0), "server available")

	var goal_handle := _send(client, 8)
	await goal_handle.completed
	expect_eq(goal_handle.get_status(), RosGoalHandle.STATUS_ABORTED, "status")


func test_goal_can_be_canceled():
	var node := make_node("actions_cancel")
	var _server := node.create_action_server("/rclgd_test/fibonacci_cancel", FIBONACCI, _execute_fibonacci)
	var client := node.create_action_client("/rclgd_test/fibonacci_cancel", FIBONACCI)
	expect(client.wait_for_server(3.0), "server available")

	var goal_handle := _send(client, 1000)
	var feedback := []
	goal_handle.feedback.connect(func(msg): feedback.append(msg))
	expect(await wait_until(func(): return not feedback.is_empty()), "goal started executing")
	goal_handle.cancel()
	await goal_handle.completed
	expect_eq(goal_handle.get_status(), RosGoalHandle.STATUS_CANCELED, "status")
