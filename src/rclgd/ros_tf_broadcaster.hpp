#ifndef ROS_TF_BROADCASTER_HPP
#define ROS_TF_BROADCASTER_HPP

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/classes/ref.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "ros_msg.hpp"


using namespace godot;

class RosTfBroadcaster : public RefCounted {
    GDCLASS(RosTfBroadcaster, RefCounted);

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
    geometry_msgs::msg::TransformStamped t;
protected:
    static void _bind_methods();

public:
    RosTfBroadcaster() = default;
    ~RosTfBroadcaster() = default;

    void setup(std::shared_ptr<rclcpp::Node> p_node);
    void send_transform(const Transform3D &p_transform, const String &p_frame_id, const String &p_parent_frame_id, bool p_is_static, const Ref<RosMsg> &p_time_msg);
};

#endif // ROS_TF_BROADCASTER_HPP