#ifndef ROS_NODE_3D_HPP
#define ROS_NODE_3D_HPP

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/engine.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "../rclgd/rclgd.hpp" // Required to access the singleton executor

using namespace godot;

class RosNode3D : public Node3D {
    GDCLASS(RosNode3D, Node3D);

private:
    static std::shared_ptr<rclcpp::Node> static_node;
    static std::unique_ptr<tf2_ros::TransformBroadcaster> static_broadcaster;
    
    String frame_id = "link";
    double publish_rate = 20.0; 
    double time_since_last_publish = 0.0;
    bool enabled = true;

    // Internal helper to ensure ROS is ready
    void _ensure_registration();

protected:
    static void _bind_methods();

public:
    RosNode3D() = default;
    ~RosNode3D() = default;

    void _enter_tree() override; // Handles auto-registration
    void _process(double delta) override;

    // Getters / Setters for Editor
    void set_frame_id(const String &p_id) { frame_id = p_id; }
    String get_frame_id() const { return frame_id; }
    void set_publish_rate(double p_rate) { publish_rate = p_rate; }
    double get_publish_rate() const { return publish_rate; }
};

#endif