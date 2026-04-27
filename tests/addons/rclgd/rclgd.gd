@tool
extends EditorPlugin

const DEST_FOLDER = "res://addons/rclgd/gen"
const TYPE_BLACKLIST = [
	"vision_msgs/msg/VisionClass", # Has a gdescript reserved keyword class_name
	"example_interfaces/msg/WString"
]

func _enter_tree():
	# Add a menu item to the "Project" menu
	add_tool_menu_item("Generate ROS2 Support Scripts", _generate_scripts)


func _exit_tree():
	remove_tool_menu_item("Generate ROS2 Support Scripts")

func _enable_plugin() -> void:
	# Add rclc autoload
	add_autoload_singleton("ROS", "res://addons/rclgd/ros_init.gd")
	print("rclgd: Autoload registered.")
	
func _disable_plugin() -> void:
	remove_autoload_singleton("ROS")
	print("RclGD: Autoload removed.")
	
func _generate_scripts():
	# 1. Ask ROS for all available interfaces (Optional but recommended)
	var output = []
	OS.execute("ros2", ["interface", "list", "-m"], output)
	
	if output.is_empty():
		printerr("Could not find ROS2. Make sure your environment is sourced.")
		return

	print("Generating ROS2 types...")
	var interfaces = output[0].split("\n")
	
	for line in interfaces:
		var type_name = line.strip_edges()
		if type_name.is_empty() or not "/" in type_name:
			continue
		if type_name in TYPE_BLACKLIST:
			print("Skipping blacklisted type: ", type_name)
			continue
		# Call your C++ static method
		# This will recurse and generate dependencies (like Header) automatically
		RosMsg.gen_editor_support(type_name, DEST_FOLDER)

	# 2. Tell Godot to scan the new files so class_names show up
	EditorInterface.get_resource_filesystem().scan()
	print("Generation finished. You may need to restart the editor for autocomplete.")
