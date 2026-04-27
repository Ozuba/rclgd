extends Node

func _ready():
	# 1. Wait for everything to settle
	await get_tree().process_frame
	
	print("\n>>> STARTING RCLGD SYSTEM TESTS <<<\n")
	
	var total := 0
	var failed := 0
	
	# 2. Iterate through all child nodes
	for child in get_children():
		if child is Test:
			total += 1
			print("Running: ", child.name)
			var success = await child.run_test()
			if not success:
				failed += 1
			print("Result: %s\n" % ["PASS" if success else "FAIL"])

	# 3. Report and Exit
	print(">>> TEST SUMMARY: %d/%d Passed <<<" % [total - failed, total])
	
	# Exit 0 if all pass, 1 if any fail (Crucial for CI/CD)
	get_tree().quit(0 if failed == 0 else 1)
