#include "ros_tf_listener_3d.hpp"

void RosTfListener3D::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_source_frame", "frame"), &RosTfListener3D::set_source_frame);
    ClassDB::bind_method(D_METHOD("get_source_frame"), &RosTfListener3D::get_source_frame);
    ClassDB::bind_method(D_METHOD("set_target_frame", "frame"), &RosTfListener3D::set_target_frame);
    ClassDB::bind_method(D_METHOD("get_target_frame"), &RosTfListener3D::get_target_frame);

    ADD_PROPERTY(PropertyInfo(Variant::STRING, "source_frame"), "set_source_frame", "get_source_frame");
    ADD_PROPERTY(PropertyInfo(Variant::STRING, "target_frame"), "set_target_frame", "get_target_frame");
}

void RosTfListener3D::_enter_tree() {
    // Connect to the engine pulse as soon as we exist in the world
    SceneTree *tree = Object::cast_to<SceneTree>(Engine::get_singleton()->get_main_loop());
    if (tree && !tree->is_connected("process_frame", callable_mp(this, &RosTfListener3D::_on_frame))) {
        tree->connect("process_frame", callable_mp(this, &RosTfListener3D::_on_frame));
    }
}

void RosTfListener3D::_exit_tree() {
    // Clean up connections to avoid memory leaks or "ghost" calls
    SceneTree *tree = Object::cast_to<SceneTree>(Engine::get_singleton()->get_main_loop());
    if (tree && tree->is_connected("process_frame", callable_mp(this, &RosTfListener3D::_on_frame))) {
        tree->disconnect("process_frame", callable_mp(this, &RosTfListener3D::_on_frame));
    }
}


void RosTfListener3D::_on_frame() {
    if (!enabled) return;

    auto rclgd_ptr = rclgd::get_singleton();
    if (!rclgd_ptr) return;

    auto buffer = rclgd_ptr->get_tf_buffer();
    if (!buffer) return;

    try {
        auto ts = buffer->lookupTransform(
            source_frame.utf8().get_data(),
            target_frame.utf8().get_data(),
            tf2::TimePointZero
        );

        // --- USING YOUR COORDINATE CONVERSION ---
        // Translation:
        // Godot X = ROS Y
        // Godot Y = ROS Z
        // Godot Z = ROS X
        Vector3 g_pos(
            ts.transform.translation.y, 
            ts.transform.translation.z, 
            ts.transform.translation.x  
        );

        // Rotation:
        // Swizzling the components to match the translation logic
        Quaternion q(
            ts.transform.rotation.x,
            ts.transform.rotation.y,
            ts.transform.rotation.z,
            ts.transform.rotation.w
        );
        
        // Applying your swizzle to the rotation components
        // x_g = y_r, y_g = z_r, z_g = x_r
        Quaternion g_q(q.y, q.z, q.x, q.w);

        set_global_transform(Transform3D(Basis(g_q), g_pos));

    } catch (const tf2::TransformException &ex) {
        // Silently skip if frame is missing
    }
}