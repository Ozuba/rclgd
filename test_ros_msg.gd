extends SceneTree

func _init():
	print("--- ROS 2 Message Validation ---")
	test_object_array_edit_in_place()
	quit()

func test_object_array_edit_in_place():
	# 1. Create a PoseArray (contains an array of Compound objects)
	var msg = RosMsg.from_type("geometry_msgs/PoseArray")
	
	# 2. Initialize array with 2 empty poses
	# Calling set_member/property trigger's the ROS buffer resize
	msg.poses = [null, null] 
	
	print("Initial X: ", msg.poses[0].position.x) # Should be 0.0
	
	# 3. GET A REFERENCE TO AN ELEMENT
	var first_pose = msg.poses[0]
	
	# 4. MODIFY IN PLACE
	first_pose.position.x = 123.45
	
	# 5. VERIFY
	if msg.poses[0].position.x == 123.45:
		print("✅ PASS: Edit-in-place detected in main message!")
	else:
		print("❌ FAIL: Changes were not reflected in the main message.")

	# 6. VERIFY MEMORY SAFETY (Aliasing)
	var second_pose = msg.poses[1]
	msg = null # Kill the parent message
	
	# If aliasing works, second_pose still has a valid shared_ptr to the memory
	second_pose.position.y = 55.5
	if second_pose.position.y == 55.5:
		print("✅ PASS: Aliased child survived parent destruction!")
	else:
		print("❌ FAIL: Child memory corrupted.")
