extends Test

var tf_received = false

func _execute():
	var node = RosNode.new()
	node.init("test_tf_node")
	
	# Broadcaster
	var broadcaster = node.create_tf_broadcaster()
	
	# Listener
	var listener = node.create_tf_listener()
	
	# Wait a bit for discovery
	await get_tree().create_timer(0.01).timeout
	
	var timeout = 2.0
	var start_time = Time.get_ticks_msec()
	
	while not tf_received:
		# Publish transform repeatedly to ensure it's received
		broadcaster.send_transform(Transform3D(Basis(), Vector3(1, 2, 3)), "child_frame", "parent_frame", false, node.now())
		
		# Try to look up the transform (null means the lookup failed)
		var transform = listener.lookup_transform("parent_frame", "child_frame")
		if transform != null:
			var origin = transform.origin
			if abs(origin.x - 1.0) < 0.001 and abs(origin.y - 2.0) < 0.001 and abs(origin.z - 3.0) < 0.001:
				tf_received = true
				break
		
		var current_time = (Time.get_ticks_msec() - start_time) / 1000.0
		if current_time >= timeout:
			break
			
		await get_tree().create_timer(0.01).timeout
		
	assert_equal(tf_received, true, "TF received successfully")
