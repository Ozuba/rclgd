# pragma once
#include <godot_cpp/classes/editor_plugin.hpp>

using namespace godot;

class RosEditorPlugin : public EditorPlugin {
    GDCLASS(RosEditorPlugin, EditorPlugin)

protected:
    static void _bind_methods();

public:
    void _enter_tree() override;
    void _exit_tree() override;

    // The function that gets called by the menu
    void gen_editor_interfaces();
};

