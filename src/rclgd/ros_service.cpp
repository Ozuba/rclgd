#include "ros_service.hpp"
#include "rclgd.hpp"

void RosService::_bind_methods()
{
    // No specific methods needed for GDScript to call on the server object itself
    // as the logic is handled by the initial setup and the callback.
}

/*
 * THREADING WARNING:
 * Unlike subscriptions and timers (which defer to the main thread), the
 * service callback MUST run synchronously so the response is populated
 * before it is sent back to the client. With the default separate-thread
 * executor this means the GDScript callback executes on a ROS executor
 * thread, concurrently with the main thread. Keep service callbacks
 * self-contained: don't touch the SceneTree or other nodes' state from
 * them without your own synchronization (e.g. call_deferred for side
 * effects that don't affect the response).
 * In physics-synchronous mode (use_separate_thread:=false) callbacks run
 * on the main thread and none of this applies.
 */
void RosService::setup(std::shared_ptr<rclcpp::Node> p_node, const String &p_srv_name, const String &p_srv_type, const Callable &p_callback, const Ref<RosQoS> &p_qos)
{
    callback_ = p_callback;

    // BabelFish takes an rmw_qos_profile_t for services; fall back to the ROS
    // service defaults when no profile was provided.
    rmw_qos_profile_t qos_profile = p_qos.is_valid()
        ? p_qos->get_qos().get_rmw_qos_profile()
        : rmw_qos_profile_services_default;

    // Create the service via BabelFish
    try
    {
        service_ = rclgd::get_singleton()->get_fish().create_service(
            *p_node,
            p_srv_name.utf8().get_data(),
            p_srv_type.utf8().get_data(),
            [this](const ros_babel_fish::CompoundMessage::SharedPtr req_in,
                   ros_babel_fish::CompoundMessage::SharedPtr res_out)
            {
                // Wrap the incoming raw buffers into Godot RosMsg objects
                Ref<RosMsg> godot_req;
                godot_req.instantiate();
                godot_req->init_babel(req_in);

                Ref<RosMsg> godot_res;
                godot_res.instantiate();
                godot_res->init_babel(res_out);

                // Execute GDScript callback immediately (synchronously)
                // Function signature: func my_srv(req: RosMsg, res: RosMsg):
                if (callback_.is_valid())
                {
                    // Call synchronously so the response is populated before returning to the client
                    callback_.call(godot_req, godot_res);
                }
            },
            qos_profile);
    }
    catch (const std::exception &e)
    {
        RCLGD_FAIL_MSG(vformat("RCLGD: Failed to create service '%s' (%s): %s",
                               p_srv_name, p_srv_type, e.what()));
    }
}