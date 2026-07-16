#include "ros_tf_listener.hpp"
#include "utils/ros_tf_utils.hpp"

void RosTfListener::_bind_methods() {
    ClassDB::bind_method(D_METHOD("lookup_transform", "target_frame", "source_frame", "timeout_sec"),
                         &RosTfListener::lookup_transform, DEFVAL(0.0));
    ClassDB::bind_method(D_METHOD("can_transform", "target_frame", "source_frame"), &RosTfListener::can_transform);
}

void RosTfListener::setup(std::shared_ptr<rclcpp::Node> p_node) {
    node_ = p_node;
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    // Subscribe through the owning node (spun by the global executor) instead
    // of letting TransformListener spawn its own internal node and thread.
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, false);
}

Variant RosTfListener::lookup_transform(const String &p_target_frame, const String &p_source_frame, double p_timeout_sec) {
    if (!tf_buffer_ || !node_ || !rclcpp::ok()) return Variant();

    try {
        // 1. Resolve frames (expands ~ to the node namespace)
        std::string target = RclgdUtils::resolve_frame(node_, p_target_frame);
        std::string source = RclgdUtils::resolve_frame(node_, p_source_frame);

        // 2. Lookup the transform (TimePointZero = latest available)
        auto t = tf_buffer_->lookupTransform(target, source, tf2::TimePointZero,
                                             tf2::durationFromSec(p_timeout_sec));

        // 3. Map ROS convention to Godot convention (shared helper)
        return RclgdUtils::ros_to_godot_transform(t.transform);

    } catch (const tf2::TransformException &e) {
        // Lookup failures are routine while frames are being published;
        // return null so the caller can tell, without breaking into the debugger.
        return Variant();
    }
}

bool RosTfListener::can_transform(const String &p_target_frame, const String &p_source_frame) const {
    if (!tf_buffer_ || !node_ || !rclcpp::ok()) return false;
    std::string target = RclgdUtils::resolve_frame(node_, p_target_frame);
    std::string source = RclgdUtils::resolve_frame(node_, p_source_frame);
    return tf_buffer_->canTransform(target, source, tf2::TimePointZero);
}
