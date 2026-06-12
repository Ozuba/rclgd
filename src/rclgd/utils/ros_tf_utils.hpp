#ifndef ROS_TF_UTILS_HPP
#define ROS_TF_UTILS_HPP

#include <rclcpp/rclcpp.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/vector3.hpp>
#include <godot_cpp/variant/quaternion.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <string>

namespace RclgdUtils {
    // Shared logic for resolving ~frame_id
    static std::string resolve_frame(std::shared_ptr<rclcpp::Node> n, const godot::String &p_id) {
        std::string s = p_id.utf8().get_data();
        if (s.empty()) return "";

        // If it starts with ~, expand the node namespace (e.g. "/car" + "/" + "lidar")
        if (s[0] == '~' && n)
            s = std::string(n->get_namespace()) + "/" + s.substr(1);

        // Strip ANY leading slashes (TF2 does not tolerate them)
        size_t start = s.find_first_not_of('/');
        return (start == std::string::npos) ? "" : s.substr(start);
    }

    /*
     * Coordinate convention mapping between Godot and ROS.
     * Both are right-handed, so a pure axis permutation suffices:
     *   ROS +X (forward) <-> Godot -Z (forward)
     *   ROS +Y (left)    <-> Godot -X (left)
     *   ROS +Z (up)      <-> Godot +Y (up)
     * These helpers are the single source of truth: the TF broadcaster,
     * the TF listener and the GDScript-facing rclgd helpers all use them.
     */
    static inline godot::Vector3 godot_to_ros_vector(const godot::Vector3 &v) {
        return godot::Vector3(-v.z, -v.x, v.y);
    }

    static inline godot::Vector3 ros_to_godot_vector(const godot::Vector3 &v) {
        return godot::Vector3(-v.y, v.z, -v.x);
    }

    static inline godot::Quaternion godot_to_ros_quat(const godot::Quaternion &q) {
        return godot::Quaternion(-q.z, -q.x, q.y, q.w);
    }

    static inline godot::Quaternion ros_to_godot_quat(const godot::Quaternion &q) {
        return godot::Quaternion(-q.y, q.z, -q.x, q.w);
    }

    static inline geometry_msgs::msg::Transform godot_to_ros_transform(const godot::Transform3D &g_xform) {
        geometry_msgs::msg::Transform t;
        godot::Vector3 pos = godot_to_ros_vector(g_xform.origin);
        t.translation.x = pos.x;
        t.translation.y = pos.y;
        t.translation.z = pos.z;

        godot::Quaternion q = godot_to_ros_quat(g_xform.basis.get_quaternion());
        t.rotation.x = q.x;
        t.rotation.y = q.y;
        t.rotation.z = q.z;
        t.rotation.w = q.w;
        return t;
    }

    static inline godot::Transform3D ros_to_godot_transform(const geometry_msgs::msg::Transform &t) {
        godot::Transform3D g_xform;
        g_xform.origin = ros_to_godot_vector(
            godot::Vector3(t.translation.x, t.translation.y, t.translation.z));
        g_xform.basis = godot::Basis(ros_to_godot_quat(
            godot::Quaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)));
        return g_xform;
    }
}

#endif
