#include "ros_tf_broadcaster.hpp"
#include "utils/ros_tf_utils.hpp"

void RosTfBroadcaster::_bind_methods() {
    ClassDB::bind_method(D_METHOD("send_transform", "transform", "frame_id", "parent_frame_id", "is_static", "timestamp"), 
                         &RosTfBroadcaster::send_transform, 
                         DEFVAL(false), 
                         DEFVAL(Variant()));
}

void RosTfBroadcaster::setup(std::shared_ptr<rclcpp::Node> p_node) {
    node_ = p_node;
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
    static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(node_);
}

void RosTfBroadcaster::send_transform(const Transform3D &p_transform, const String &p_frame_id, const String &p_parent_frame_id, bool p_is_static, const Ref<RosMsg> &p_time_msg) {
    if (!node_ || !rclcpp::ok()) return;
    
    geometry_msgs::msg::TransformStamped t;

    // --- Timestamp Acquisition ---
    if (p_time_msg.is_valid()) {
        int32_t sec = p_time_msg->get("sec");
        uint32_t nanosec = p_time_msg->get("nanosec");
        t.header.stamp = rclcpp::Time(sec, nanosec, node_->get_clock()->get_clock_type());
    } else {
        t.header.stamp = node_->now();
    }

    t.header.frame_id = RclgdUtils::resolve_frame(node_, p_parent_frame_id);
    t.child_frame_id = RclgdUtils::resolve_frame(node_, p_frame_id);                        

    // --- Position Mapping (Godot -> ROS 2 Vehicle Frame) ---
    Vector3 pos = p_transform.origin;
    t.transform.translation.x = -pos.z; 
    t.transform.translation.y = -pos.x; 
    t.transform.translation.z =  pos.y; 

    // --- Rotation Mapping (Godot -> ROS 2 Vehicle Frame) ---
    Quaternion q = p_transform.basis.get_quaternion();
    t.transform.rotation.x = -q.z; 
    t.transform.rotation.y = -q.x; 
    t.transform.rotation.z =  q.y; 
    t.transform.rotation.w =  q.w; 

    // --- Broadcast Target Selection ---
    if (p_is_static) {
        static_broadcaster_->sendTransform(t);
    } else {
        broadcaster_->sendTransform(t);
    }
}