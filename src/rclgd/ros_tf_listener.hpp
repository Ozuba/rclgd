#ifndef ROS_TF_LISTENER_HPP
#define ROS_TF_LISTENER_HPP

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/transform3d.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "rclgd.hpp"



using namespace godot;

class RosTfListener : public RefCounted {
    GDCLASS(RosTfListener, RefCounted);

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

protected:
    static void _bind_methods();

public:
    RosTfListener() = default;
    ~RosTfListener() = default;

    void setup(std::shared_ptr<rclcpp::Node> p_node);
    Transform3D lookup_transform(const String &p_target_frame, const String &p_source_frame);
    bool can_transform(const String &p_target_frame, const String &p_source_frame) const;
};

#endif // ROS_TF_LISTENER_HPP