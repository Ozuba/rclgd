#include "ros_tf_broadcaster_3d.hpp"
#include <godot_cpp/variant/utility_functions.hpp>

void RosTfBroadcaster3D::_enter_tree()
{
    SceneTree *tree = Object::cast_to<SceneTree>(Engine::get_singleton()->get_main_loop());
    if (tree)
    {
        tree->connect("physics_frame", callable_mp(this, &RosTfBroadcaster3D::_on_physics_tick));
    }
}
void RosTfBroadcaster3D::_exit_tree()
{
    SceneTree *tree = Object::cast_to<SceneTree>(Engine::get_singleton()->get_main_loop());
    if (tree)
    {
        tree->disconnect("physics_frame", callable_mp(this, &RosTfBroadcaster3D::_on_physics_tick));
    }
}

void RosTfBroadcaster3D::_on_physics_tick()
{
    if (!enabled || !rclgd::get_singleton()->ok())
        return;

    double delta = get_physics_process_delta_time();

    time_since_last_publish += delta;
    if (time_since_last_publish < (1.0 / publish_rate))
        return;
    time_since_last_publish = 0.0;


    Transform3D target_transform;
    Node3D* parent_node = Object::cast_to<RosTfBroadcaster3D>(get_node_or_null(parent_node_path));
    if (parent_node)
    {
        // Option A: Node is selected - Calculate Relative
        target_transform = parent_node->get_global_transform().affine_inverse() * get_global_transform();
    }
    else
    {
        // No node selected - Use Global Transform with provided parent_frame_id
        target_transform = get_global_transform();
    }

    // Generate the transfrom
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = rclgd::get_singleton()->get_rclgd_node()->now();
    t.header.frame_id = parent_frame_id.utf8().get_data();
    t.child_frame_id = frame_id.utf8().get_data();

    // 1. Position: Standard X-Forward Mapping
    Vector3 pos = target_transform.origin;
    t.transform.translation.x = pos.z; // Godot Forward (+Z) -> ROS X
    t.transform.translation.y = pos.x; // Godot Right (+X)
    t.transform.translation.z = pos.y; // Godot Up (+Y) -> ROS Z

    // 2. Rotation: Vector-by-Vector Mapping
    // We extract Godot's local axes and map them to ROS 2's axes
    Vector3 g_right = target_transform.basis.get_column(0);   // +X
    Vector3 g_up = target_transform.basis.get_column(1);      // +Y
    Vector3 g_forward = target_transform.basis.get_column(2); // +Z

    // Construct a new ROS Basis
    Basis ros_basis;
    ros_basis.set_column(0, Vector3(g_forward.z, g_forward.x, g_forward.y));
    ros_basis.set_column(1, Vector3(g_right.z, g_right.x, g_right.y));
    ros_basis.set_column(2, Vector3(g_up.z, g_up.x, g_up.y));

    // Convert to Quaternion
    Quaternion q = ros_basis.get_quaternion();
    t.transform.rotation.x = q.x;
    t.transform.rotation.y = q.y;
    t.transform.rotation.z = q.z;
    t.transform.rotation.w = q.w;

    rclgd::get_singleton()->get_tf_broadcaster()->sendTransform(t);
}

void RosTfBroadcaster3D::_bind_methods()
{
    ClassDB::bind_method(D_METHOD("set_frame_id", "id"), &RosTfBroadcaster3D::set_frame_id);
    ClassDB::bind_method(D_METHOD("get_frame_id"), &RosTfBroadcaster3D::get_frame_id);
    ClassDB::bind_method(D_METHOD("set_parent_frame_id", "p_id"), &RosTfBroadcaster3D::set_parent_frame_id);
    ClassDB::bind_method(D_METHOD("get_parent_frame_id"), &RosTfBroadcaster3D::get_parent_frame_id);
    ClassDB::bind_method(D_METHOD("set_parent_node_path", "p_path"), &RosTfBroadcaster3D::set_parent_node_path);
    ClassDB::bind_method(D_METHOD("get_parent_node_path"), &RosTfBroadcaster3D::get_parent_node_path);

    ClassDB::bind_method(D_METHOD("set_publish_rate", "hz"), &RosTfBroadcaster3D::set_publish_rate);
    ClassDB::bind_method(D_METHOD("get_publish_rate"), &RosTfBroadcaster3D::get_publish_rate);

    ADD_PROPERTY(PropertyInfo(Variant::STRING, "frame_id"), "set_frame_id", "get_frame_id");
    ADD_PROPERTY(PropertyInfo(Variant::STRING, "parent_frame_id"), "set_parent_frame_id", "get_parent_frame_id");
    ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "parent_node_path", PROPERTY_HINT_NODE_TYPE, "Node3D"), "set_parent_node_path", "get_parent_node_path"); //Restrict to Node3d
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "publish_rate"), "set_publish_rate", "get_publish_rate");
}