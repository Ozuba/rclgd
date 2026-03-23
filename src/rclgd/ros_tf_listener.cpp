#include "ros_tf_listener.hpp"

void RosTfListener::_bind_methods() {
    ClassDB::bind_method(D_METHOD("lookup_transform", "target_frame", "source_frame"), &RosTfListener::lookup_transform);
}

void RosTfListener::setup(std::shared_ptr<rclcpp::Node> p_node) {
    node_ = p_node;
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}


Transform3D RosTfListener::lookup_transform(const String &p_target_frame, const String &p_source_frame) {
    Transform3D g_xform;
    if (!tf_buffer_) return g_xform;

       auto resolve_frame = [this](const String &p_id) -> std::string {
        std::string id_str = p_id.utf8().get_data();
        if (id_str.empty()) return "";

        // 1. Check for the tilde toggle
        if (id_str[0] == '~') {
            std::string ns = node_->get_namespace();
            
            // Strip the '~' and return the namespaced frame
            std::string pure_id = id_str.substr(1);
            
            if (ns.empty()) return pure_id;
            return ns + "/" + pure_id;
        }

        // 2. No tilde? Return as a global frame (e.g., "map" or "odom")
        return id_str;
    };

    try {
        auto t = tf_buffer_->lookupTransform(
            resolve_frame(p_target_frame),
            resolve_frame(p_source_frame),
            tf2::TimePointZero
        );

        // ... [Rest of your Inverse Position/Rotation Mapping] ...
        g_xform.origin.z = t.transform.translation.x;
        g_xform.origin.x = t.transform.translation.y;
        g_xform.origin.y = t.transform.translation.z;
        // (Inverse Rotation logic goes here)
    } catch (const tf2::TransformException &ex) {}

    return g_xform;
}