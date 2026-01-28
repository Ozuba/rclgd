#ifndef ROS_NODE_3D_HPP
#define ROS_NODE_3D_HPP

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/variant/callable_method_pointer.hpp>
#include <godot_cpp/classes/scene_tree.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "rclgd/rclgd.hpp" 

using namespace godot;

class RosTfBroadcaster3D : public Node3D {
    GDCLASS(RosTfBroadcaster3D, Node3D);

private:
    String frame_id = "link";
    double publish_rate = 20.0; 
    double time_since_last_publish = 0.0;
    bool enabled = true;

    void _ensure_registration();

protected:
    static void _bind_methods();

public:
    RosTfBroadcaster3D() = default;
    ~RosTfBroadcaster3D() = default;

    void _enter_tree() override; 
    void _exit_tree() override; 

    void _on_physics_tick();

    // Getters / Setters
    void set_frame_id(const String &p_id) { frame_id = p_id; }
    String get_frame_id() const { return frame_id; }
    void set_publish_rate(double p_rate) { publish_rate = p_rate; }
    double get_publish_rate() const { return publish_rate; }
};

#endif