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

        // 3. Inverse Position Mapping (ROS -> Godot)
        // Broadcaster: ros.x=pos.z, ros.y=pos.x, ros.z=pos.y
        // Listener:    pos.x=ros.y, pos.y=ros.z, pos.z=ros.x
        g_xform.origin.x = t.transform.translation.y;
        g_xform.origin.y = t.transform.translation.z;
        g_xform.origin.z = t.transform.translation.x;

        // 4. Inverse Rotation Mapping (ROS -> Godot)
        // First, get the basis from the ROS quaternion
        Quaternion q(
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w
        );
        Basis ros_basis(q);

        // Extract the ROS columns
        Vector3 r_c0 = ros_basis.get_column(0); // Shuffled Forward
        Vector3 r_c1 = ros_basis.get_column(1); // Shuffled Right
        Vector3 r_c2 = ros_basis.get_column(2); // Shuffled Up

        // Helper to un-shuffle a vector from (z, x, y) back to (x, y, z)
        auto unshuffle = [](const Vector3 &v) {
            return Vector3(v.y, v.z, v.x);
        };

        // Reconstruct Godot Basis
        // Broadcaster mapping: 
        // ros_c0 = shuffled g_forward | ros_c1 = shuffled g_right | ros_c2 = shuffled g_up
        g_xform.basis.set_column(0, unshuffle(r_c1)); // Godot Right
        g_xform.basis.set_column(1, unshuffle(r_c2)); // Godot Up
        g_xform.basis.set_column(2, unshuffle(r_c0)); // Godot Forward

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