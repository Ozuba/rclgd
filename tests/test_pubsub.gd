extends Test

var received_msg = null

func _execute():
	var node = RosNode.new()
	node.init("test_loopback_node")
	
	# Subscriber
	var sub = node.create_subscription("/test_topic", "std_msgs/msg/String", func(msg):
		received_msg = msg.data
	)
	
	# Publisher
	var pub = node.create_publisher("/test_topic", "std_msgs/msg/String")
	
	# Prepare message
	var out_msg = RosMsg.from_type("std_msgs/msg/String")
	out_msg.data = "hey_from_godot"
	
	pub.publish(out_msg)
	
	# Wait for message
	var timeout = 2.0
	var start_time = Time.get_ticks_msec()
	
	while received_msg == null:
		var current_time = (Time.get_ticks_msec() - start_time) / 1000.0
		if current_time >= timeout:
			break
		await get_tree().create_timer(0.01).timeout
	
	assert_equal(received_msg, "hey_from_godot", "Received message content")
