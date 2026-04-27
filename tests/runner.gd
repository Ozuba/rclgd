extends Node

func _ready():
	# 1. Wait for everything to settle
	if not rclgd.ok(): rclgd.init([])
	await get_tree().process_frame
	
	print_rich("\n[b][color=cyan]>>> STARTING RCLGD SYSTEM TESTS <<<[/color][/b]\n")
	
	var total := 0
	var failed := 0
	
	# 2. Iterate through all child nodes
	for child in get_children():
		if child is Test:
			total += 1
			print_rich("[color=yellow]Running:[/color] [b]%s[/b]" % child.name)
			var success = await child.run_test()
			if not success:
				failed += 1
			
			if success:
				print_rich("Result: [b][color=green]PASS[/color][/b]\n")
			else:
				print_rich("Result: [b][color=red]FAIL[/color][/b]\n")

	# 3. Report and Exit
	if failed == 0:
		print_rich("[b][color=green]>>> TEST SUMMARY: %d/%d Passed <<<[/color][/b]" % [total - failed, total])
	else:
		print_rich("[b][color=red]>>> TEST SUMMARY: %d/%d Passed <<<[/color][/b]" % [total - failed, total])
	
	# Free all test nodes so their ROS objects are destroyed BEFORE rclgd.shutdown()
	for child in get_children():
		child.free()
		
	# Exit 0 if all pass, 1 if any fail (Crucial for CI/CD)
	rclgd.shutdown()
	get_tree().quit(0 if failed == 0 else 1)
