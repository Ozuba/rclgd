#ifndef ROS_TF_LISTENER_3D_HPP
#define ROS_TF_LISTENER_3D_HPP

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/variant/callable_method_pointer.hpp>
#include <godot_cpp/classes/scene_tree.hpp>

#include <tf2_ros/buffer.h>
#include "rclgd/rclgd.hpp" 

using namespace godot;

class RosTfListener3D : public Node3D {
    GDCLASS(RosTfListener3D, Node3D)

private:
    String source_frame = "map";
    String target_frame = "base_link";
    bool enabled = true;

protected:
    static void _bind_methods();

public:
    void _enter_tree() override; 
    void _exit_tree() override; 

    //Transform Updater
    void _on_frame();


    // Getters/Setters for Godot Inspector
    void set_source_frame(const String &p_frame) { source_frame = p_frame; }
    String get_source_frame() const { return source_frame; }
    
    void set_target_frame(const String &p_frame) { target_frame = p_frame; }
    String get_target_frame() const { return target_frame; }
};

#endif