#include "ros_editor_plugin.hpp"
#include <godot_cpp/variant/utility_functions.hpp>

void RosEditorPlugin::_bind_methods() {
    ClassDB::bind_method(D_METHOD("gen_editor_interfaces"), &RosEditorPlugin::gen_editor_interfaces);
}

void RosEditorPlugin::_enter_tree() {
    // Add the menu item to Project -> Tools -> ROS 2 Discover Interfaces
    add_tool_menu_item("ROS 2 Interface Gen Support", Callable(this, "gen_editor_interfaces"));
}

void RosEditorPlugin::_exit_tree() {
    // Cleanup: Remove the menu item when the plugin is disabled
    remove_tool_menu_item("ROS 2 Discover Interfaces");
}

void RosEditorPlugin::gen_editor_interfaces() {
    UtilityFunctions::print("Starting ROS 2 igeneration...");
    
   
    
    UtilityFunctions::print("Generation complete!");
}