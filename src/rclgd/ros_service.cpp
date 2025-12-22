#include "ros_service.hpp"
#include "rclgd.hpp"

void RosService::_bind_methods() {
    // No specific methods needed for GDScript to call on the server object itself
    // as the logic is handled by the initial setup and the callback.
}

void RosService::setup(std::shared_ptr<rclcpp::Node> p_node, const String &p_srv_name, const String &p_srv_type, const Callable &p_callback) {
    callback_ = p_callback;

    // Create the service via BabelFish
    service_ = rclgd::get_singleton()->get_fish().create_service(
        *p_node,
        p_srv_name.utf8().get_data(),
        p_srv_type.utf8().get_data(),
        [this](const ros_babel_fish::CompoundMessage::SharedPtr req_in, 
               ros_babel_fish::CompoundMessage::SharedPtr res_out) {
            
            // Wrap the incoming raw buffers into Godot RosMsg objects
            Ref<RosMsg> godot_req; 
            godot_req.instantiate(); 
            godot_req->init_babel(req_in);

            Ref<RosMsg> godot_res; 
            godot_res.instantiate(); 
            godot_res->init_babel(res_out);

            // Execute GDScript callback immediately (synchronously)
            // Function signature: func my_srv(req: RosMsg, res: RosMsg):
            if (callback_.is_valid()) {
                callback_.call(godot_req, godot_res);
            }
        }
    );
}