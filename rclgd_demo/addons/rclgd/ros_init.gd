extends Node

func _enter_tree() -> void:
	# 1. Guard against running ROS logic in the Editor.
	if Engine.is_editor_hint():
		return

	# 2. Initialize the ROS 2 context.
	var args = OS.get_cmdline_args()
	
	# Assuming 'rclgd' is the name registered in register_types.cpp
	if rclgd:
		rclgd.init(args)
		print("[ROS] Context initialized and Executor started.")
	else:
		push_error("[ROS] rclgd singleton not found! Is the GDExtension loaded?")

func _exit_tree() -> void:
	# 3. Clean up when the game or scene closes.
	if not Engine.is_editor_hint() and rclgd:
		rclgd.shutdown()
		print("[ROS] Context shut down safely.")
