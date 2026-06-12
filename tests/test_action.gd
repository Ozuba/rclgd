extends Test

var feedback_count := 0

func _execute():
	var node = RosNode.new()
	node.init("test_action_node")

	# Server: computes a Fibonacci sequence up to goal.order, publishing
	# feedback at every step.
	var server = node.create_action_server("test_fibonacci", "example_interfaces/action/Fibonacci",
		func(goal_handle):
			var sequence: Array = [0, 1]
			for i in range(2, goal_handle.get_goal().order):
				if goal_handle.is_cancel_requested():
					var cancel_result = goal_handle.create_result()
					cancel_result.sequence = sequence
					goal_handle.canceled(cancel_result)
					return
				sequence.append(sequence[i - 1] + sequence[i - 2])
				var fb = goal_handle.create_feedback()
				fb.sequence = sequence
				goal_handle.publish_feedback(fb)
				# Yield between steps (like a real long-running goal) so the
				# feedback is delivered before the result settles the goal.
				await get_tree().create_timer(0.01).timeout
			var result = goal_handle.create_result()
			result.sequence = sequence
			goal_handle.succeed(result)
	)

	# Client
	var client = node.create_action_client("test_fibonacci", "example_interfaces/action/Fibonacci")

	var is_ready = client.wait_for_server(2.0)
	assert_equal(is_ready, true, "Action server should be ready")
	if not is_ready:
		return

	# Send a goal and track feedback
	var goal = client.create_goal()
	goal.order = 8
	var goal_handle = client.send_goal(goal)
	goal_handle.feedback.connect(func(_msg): feedback_count += 1)

	# Await completion
	await goal_handle.completed

	assert_equal(goal_handle.is_accepted(), true, "Goal should have been accepted")
	assert_equal(goal_handle.get_status(), RosGoalHandle.STATUS_SUCCEEDED, "Goal should have succeeded")
	assert_equal(goal_handle.get_result() != null, true, "Result should not be null")
	if goal_handle.get_result() != null:
		var seq = goal_handle.get_result().sequence
		assert_equal(seq.size(), 8, "Sequence should have 8 elements")
		if seq.size() == 8:
			assert_equal(int(seq[7]), 13, "8th Fibonacci number should be 13")
	assert_equal(feedback_count > 0, true, "Feedback should have been received")
