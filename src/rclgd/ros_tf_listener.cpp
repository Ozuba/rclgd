#include "ros_tf_listener.hpp"
#include "utils/ros_tf_utils.hpp"

void RosTfListener::_bind_methods() {
    ClassDB::bind_method(D_METHOD("lookup_transform", "target_frame", "source_frame"), &RosTfListener::lookup_transform);
    ClassDB::bind_method(D_METHOD("can_transform", "target_frame", "source_frame"), &RosTfListener::can_transform);
}

void RosTfListener::setup(std::shared_ptr<rclcpp::Node> p_node) {
    node_ = p_node;
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

Transform3D RosTfListener::lookup_transform(const String &p_target_frame, const String &p_source_frame) {
    Transform3D g_xform;
    if (!tf_buffer_ || !node_ || !rclcpp::ok()) return g_xform;

    try {
        // 1. Resolve frames using your utility
        std::string target = RclgdUtils::resolve_frame(node_, p_target_frame);
        std::string source = RclgdUtils::resolve_frame(node_, p_source_frame);

        // 2. Lookup the transform (TimePointZero = latest available)
        auto t = tf_buffer_->lookupTransform(target, source, tf2::TimePointZero);

        // 3. Inverse Position Mapping (ROS 2 -> Godot Ground Vehicle Frame)
        g_xform.origin.x = -t.transform.translation.y; // ROS +Y (Left)     -> Godot -X (Left)
        g_xform.origin.y =  t.transform.translation.z; // ROS +Z (Up)       -> Godot +Y (Up)
        g_xform.origin.z = -t.transform.translation.x; // ROS +X (Forward)  -> Godot -Z (Forward)

        // 4. Inverse Rotation Mapping (ROS 2 -> Godot Ground Vehicle Frame)
        Quaternion ros_q(
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w
        );

        Quaternion godot_q;
        godot_q.x = -ros_q.y; 
        godot_q.y =  ros_q.z; 
        godot_q.z = -ros_q.x; 
        godot_q.w =  ros_q.w; 

        g_xform.basis = Basis(godot_q);

    } catch (const tf2::TransformException &e) {
        // If the transform isn't found, return Identity or log error
       // RCLGD_FAIL_MSG(vformat("RCLGD TF Lookup failed: %s", e.what()));
    }

    return g_xform;
}

bool RosTfListener::can_transform(const String &p_target_frame, const String &p_source_frame) const {
    if (!tf_buffer_ || !node_ || !rclcpp::ok()) return false;
    std::string target = RclgdUtils::resolve_frame(node_, p_target_frame);
    std::string source = RclgdUtils::resolve_frame(node_, p_source_frame);
    return tf_buffer_->canTransform(target, source, tf2::TimePointZero);
}