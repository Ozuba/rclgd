extends Test

var timer_triggered = false

func _execute():
	var node = RosNode.new()
	node.init("test_timer_node")
	
	# Create a timer
	var timer = node.create_timer(0.5, func():
		timer_triggered = true
	)
	
	# Wait for it to trigger
	var timeout = 2.0
	var start_time = Time.get_ticks_msec()
	
	while not timer_triggered:
		var current_time = (Time.get_ticks_msec() - start_time) / 1000.0
		if current_time >= timeout:
			break
			
		await get_tree().create_timer(0.01).timeout
		
	assert_equal(timer_triggered, true, "Timer should trigger within timeout")
