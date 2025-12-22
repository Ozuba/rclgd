#include "ros_node_3d.hpp"
#include <godot_cpp/variant/utility_functions.hpp>

std::shared_ptr<rclcpp::Node> RosNode3D::static_node = nullptr;
std::unique_ptr<tf2_ros::TransformBroadcaster> RosNode3D::static_broadcaster = nullptr;

void RosNode3D::_enter_tree() {
    _ensure_registration();
}

void RosNode3D::_ensure_registration() {
    // If the editor is running and we are not in "Live" mode, we might want to skip ROS init
    if (Engine::get_singleton()->is_editor_hint()) return;

    // 1. Check if the static node already exists
    if (!static_node) {
        // Look for our rclcpp singleton
        auto rclgd_singleton = rclgd::get_singleton();
        if (!rclgd_singleton || !rclcpp::ok()) return;

        // Use a default name for the internal transform node if none exists
        static_node = std::make_shared<rclcpp::Node>("godot_tf_node");
        static_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(static_node);
        
        // Self-register with the executor
        rclgd_singleton->add_node(static_node);
        UtilityFunctions::print("RosNode3D: Internal static ROS node created and registered.");
    }
}

void RosNode3D::_process(double delta) {
    if (Engine::get_singleton()->is_editor_hint()) return;
    
    // Ensure we are registered (in case rclcpp started late)
    if (!static_node) {
        _ensure_registration();
        return;
    }

    if (!enabled || !static_broadcaster) return;

    // Throttling
    time_since_last_publish += delta;
    if (time_since_last_publish < (1.0 / publish_rate)) return;
    time_since_last_publish = 0.0;

    // Transform Logic
    String parent_frame_name = "world";
    Transform3D relative_transform = get_global_transform();

    Node *p = get_parent();
    while (p) {
        RosNode3D *parent_node = Object::cast_to<RosNode3D>(p);
        if (parent_node) {
            parent_frame_name = parent_node->get_frame_id();
            relative_transform = parent_node->get_global_transform().affine_inverse() * get_global_transform();
            break;
        }
        p = p->get_parent();
    }

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = static_node->now();
    t.header.frame_id = parent_frame_name.utf8().get_data();
    t.child_frame_id = frame_id.utf8().get_data();

    // Map Godot -> ROS
    Vector3 pos = relative_transform.origin;
    t.transform.translation.x = pos.x;
    t.transform.translation.y = -pos.z;
    t.transform.translation.z = pos.y;

    Quaternion q = relative_transform.basis.get_quaternion();
    t.transform.rotation.x = q.x;
    t.transform.rotation.y = -q.z;
    t.transform.rotation.z = q.y;
    t.transform.rotation.w = q.w;

    static_broadcaster->sendTransform(t);
}

void RosNode3D::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_frame_id", "id"), &RosNode3D::set_frame_id);
    ClassDB::bind_method(D_METHOD("get_frame_id"), &RosNode3D::get_frame_id);
    ClassDB::bind_method(D_METHOD("set_publish_rate", "hz"), &RosNode3D::set_publish_rate);
    ClassDB::bind_method(D_METHOD("get_publish_rate"), &RosNode3D::get_publish_rate);

    ADD_PROPERTY(PropertyInfo(Variant::STRING, "frame_id"), "set_frame_id", "get_frame_id");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "publish_rate"), "set_publish_rate", "get_publish_rate");
}