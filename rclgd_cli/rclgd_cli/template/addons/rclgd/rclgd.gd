@tool
extends EditorPlugin

## Editor-side support for rclgd.
##
## On editor load, typed GDScript wrappers (shadow classes) are generated for
## every message package listed as a dependency in the project's package.xml.
## Generation is skipped when the dependency set is unchanged since the last
## run (stamp file); use the "Project > Tools > Regenerate ROS2 Types" menu
## item to force a full regeneration (e.g. after an rclgd update).

const GEN_DIR := "res://addons/rclgd/gen"
const STAMP_FILE := GEN_DIR + "/.generated_types"

const TYPE_BLACKLIST = [
	"vision_msgs/msg/VisionClass", # Has a gdscript reserved keyword class_name
	"example_interfaces/msg/WString"
]


func _enter_tree() -> void:
	add_tool_menu_item("Regenerate ROS2 Types", _regenerate.bind(true))
	# Auto-generate once the editor's startup scan completes — generating any
	# earlier races the scan thread and the plugin-enable machinery.
	EditorInterface.get_resource_filesystem().filesystem_changed.connect(
			_regenerate.bind(false), CONNECT_ONE_SHOT)


func _exit_tree() -> void:
	remove_tool_menu_item("Regenerate ROS2 Types")


func _enable_plugin() -> void:
	# Add rclgd autoload (initializes the ROS context at game start)
	add_autoload_singleton("ROS", "res://addons/rclgd/ros_init.gd")
	print("rclgd: Autoload registered.")


func _disable_plugin() -> void:
	remove_autoload_singleton("ROS")
	print("rclgd: Autoload removed.")


func _regenerate(force: bool) -> void:
	var types := _types_from_package_xml()
	if types.is_empty():
		if force:
			printerr("rclgd: no message packages found among the dependencies of res://package.xml")
		return

	# Skip when the interface set is unchanged since the last generation.
	var stamp := "\n".join(types)
	if not force and FileAccess.file_exists(STAMP_FILE) \
			and FileAccess.get_file_as_string(STAMP_FILE) == stamp:
		return

	print("rclgd: generating typed wrappers for %d interfaces into %s" % [types.size(), GEN_DIR])
	for t in types:
		# Recurses into nested types (Header, geometry primitives, ...)
		RosMsg.gen_editor_support(t, GEN_DIR)

	DirAccess.make_dir_recursive_absolute(GEN_DIR)
	var f := FileAccess.open(STAMP_FILE, FileAccess.WRITE)
	if f:
		f.store_string(stamp)

	# Register each generated script with the editor filesystem: update_file
	# parses the class_name into the global class cache immediately, so the
	# typed classes are usable without a project reload. No scan() needed
	# (and it would be illegal from a deferred signal handler anyway) — the
	# dock catches up on the next natural rescan.
	var fs := EditorInterface.get_resource_filesystem()
	for gen_file in DirAccess.get_files_at(GEN_DIR):
		if gen_file.ends_with(".gd"):
			fs.update_file(GEN_DIR + "/" + gen_file)


## All message types provided by the packages package.xml depends on.
func _types_from_package_xml() -> PackedStringArray:
	var types := PackedStringArray()
	if not FileAccess.file_exists("res://package.xml"):
		return types
	for dep in _deps_from_manifest("res://package.xml"):
		for t in _msgs_of_package(dep):
			if not t in TYPE_BLACKLIST and not types.has(t):
				types.append(t)
	var arr := Array(types)
	arr.sort()
	return PackedStringArray(arr)


func _deps_from_manifest(path: String) -> PackedStringArray:
	var deps := PackedStringArray()
	var parser := XMLParser.new()
	if parser.open(path) != OK:
		printerr("rclgd: could not parse " + path)
		return deps
	var tag := ""
	while parser.read() == OK:
		match parser.get_node_type():
			XMLParser.NODE_ELEMENT:
				tag = parser.get_node_name()
			XMLParser.NODE_TEXT:
				if tag in ["depend", "exec_depend", "build_depend"]:
					var dep := parser.get_node_data().strip_edges()
					if dep != "" and not deps.has(dep):
						deps.append(dep)
			XMLParser.NODE_ELEMENT_END:
				tag = ""
	return deps


## Message types of one package, read from the rosidl ament index
## (<prefix>/share/ament_index/resource_index/rosidl_interfaces/<pkg>).
## Non-interface packages (e.g. rclgd itself) simply have no marker.
func _msgs_of_package(pkg: String) -> PackedStringArray:
	var out := PackedStringArray()
	for prefix in OS.get_environment("AMENT_PREFIX_PATH").split(":", false):
		var marker := prefix + "/share/ament_index/resource_index/rosidl_interfaces/" + pkg
		if not FileAccess.file_exists(marker):
			continue
		for line in FileAccess.get_file_as_string(marker).split("\n", false):
			line = line.strip_edges()
			if line.begins_with("msg/") and (line.ends_with(".msg") or line.ends_with(".idl")):
				var t := pkg + "/msg/" + line.get_slice("/", 1).get_basename()
				if not out.has(t):
					out.append(t)
		break # first prefix providing the package wins (overlay order)
	return out
