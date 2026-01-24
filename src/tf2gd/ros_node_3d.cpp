#include "ros_node_3d.hpp"
#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/variant/callable.hpp>
#include <godot_cpp/variant/callable_method_pointer.hpp>
#include <godot_cpp/classes/scene_tree.hpp>

std::shared_ptr<rclcpp::Node> RosNode3D::static_node = nullptr;
std::unique_ptr<tf2_ros::TransformBroadcaster> RosNode3D::static_broadcaster = nullptr;

void RosNode3D::_enter_tree()
{
    _ensure_registration();
}

void RosNode3D::_ensure_registration()
{
    if (Engine::get_singleton()->is_editor_hint())
        return;

    if (!static_node)
    {
        auto rclgd_singleton = rclgd::get_singleton();
        if (!rclgd_singleton || !rclcpp::ok())
            return;

        //rclcpp::NodeOptions options;
        //options.append_parameter_override("use_sim_time", true); // Let global context handle this

        static_node = std::make_shared<rclcpp::Node>("rclgd_tf");
               
        //Global parameter frame
        static_node->declare_parameter("global_frame", "map");
    
        //Transform Publisher
        static_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(static_node);
 

        rclgd_singleton->add_node(static_node);
        UtilityFunctions::print("RosNode3D: Internal static ROS node created.");
    }

    //Register physics tick update
    SceneTree *tree = Object::cast_to<SceneTree>(Engine::get_singleton()->get_main_loop());
    if (tree)
    {
        // 2. Connect using a direct method pointer (Invisible to GDScript)
        tree->connect("physics_frame", callable_mp(this, &RosNode3D::_on_physics_tick));
    }
}

void RosNode3D::_on_physics_tick()
{
    if (Engine::get_singleton()->is_editor_hint())
        return;
    if (!enabled || !static_broadcaster)
        return;
        
    double delta = get_physics_process_delta_time();

    time_since_last_publish += delta;
    if (time_since_last_publish < (1.0 / publish_rate))
        return;
    time_since_last_publish = 0.0;

    // --- Transform Logic ---
    String parent_frame_name = String(static_node->get_parameter("global_frame").as_string().c_str());
    Transform3D relative_transform = get_global_transform();

    Node *p = get_parent();
    while (p)
    {
        RosNode3D *parent_node = Object::cast_to<RosNode3D>(p);
        if (parent_node)
        {
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

    // 1. Position: Standard X-Forward Mapping
    Vector3 pos = relative_transform.origin;
    t.transform.translation.x = pos.z; // Godot Forward (+Z) -> ROS X
    t.transform.translation.y = pos.x; // Godot Right (+X)
    t.transform.translation.z = pos.y; // Godot Up (+Y) -> ROS Z

    // 2. Rotation: Vector-by-Vector Mapping
    // We extract Godot's local axes and map them to ROS 2's axes
    Vector3 g_right = relative_transform.basis.get_column(0);   // +X
    Vector3 g_up = relative_transform.basis.get_column(1);      // +Y
    Vector3 g_forward = relative_transform.basis.get_column(2); // +Z

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

    static_broadcaster->sendTransform(t);
}

void RosNode3D::_bind_methods()
{
    ClassDB::bind_method(D_METHOD("set_frame_id", "id"), &RosNode3D::set_frame_id);
    ClassDB::bind_method(D_METHOD("get_frame_id"), &RosNode3D::get_frame_id);
    ClassDB::bind_method(D_METHOD("set_publish_rate", "hz"), &RosNode3D::set_publish_rate);
    ClassDB::bind_method(D_METHOD("get_publish_rate"), &RosNode3D::get_publish_rate);

    ADD_PROPERTY(PropertyInfo(Variant::STRING, "frame_id"), "set_frame_id", "get_frame_id");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "publish_rate"), "set_publish_rate", "get_publish_rate");
}