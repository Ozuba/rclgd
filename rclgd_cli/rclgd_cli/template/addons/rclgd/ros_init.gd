extends Node

static func _static_init() -> void:
	if Engine.is_editor_hint(): return

	var cli_args: PackedStringArray = OS.get_cmdline_user_args()
	var ros_params: PackedStringArray = []
	var prefix: String = "ros/parameters/"

	for p in ProjectSettings.get_property_list():
		var p_name: String = p.name
		if p_name.begins_with(prefix):
			var key: String = p_name.trim_prefix(prefix)
			
			# Comprobación de duplicados compatible con PackedStringArray
			var override_exists: bool = false
			for arg in cli_args:
				if key + ":=" in arg:
					override_exists = true
					break
			
			if not override_exists:
				var val = ProjectSettings.get_setting_with_override(p_name)
				var res = str(val).to_lower() if val is bool else str(val)
				ros_params.append_array(["-p", "%s:=%s" % [key, res]])

	if rclgd:
		# CLI args go first, untouched: any --ros-args section they contain keeps
		# working, and plain game args never land inside one (rclcpp rejects
		# unknown ROS args). Project-settings params get their own section.
		var final_args: PackedStringArray = cli_args.duplicate()
		if not ros_params.is_empty():
			final_args.append("--ros-args")
			final_args.append_array(ros_params)

		rclgd.init(final_args)
		print("[ROS Init Args]: ", final_args)

func _exit_tree() -> void:
	# 3. Clean up when the game or scene closes.
	if not Engine.is_editor_hint() and rclgd:
		rclgd.shutdown()
		print("[ROS] Context shut down safely.")
