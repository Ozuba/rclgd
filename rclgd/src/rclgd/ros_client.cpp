#include "ros_client.hpp"
#include "rclgd.hpp"

#include <godot_cpp/variant/callable_method_pointer.hpp>

void RosClient::_bind_methods() {
    ClassDB::bind_method(D_METHOD("wait_for_service", "timeout_sec"), &RosClient::wait_for_service);
    ClassDB::bind_method(D_METHOD("wait_for_service_async", "timeout_sec"), &RosClient::wait_for_service_async);
    ClassDB::bind_method(D_METHOD("create_request"), &RosClient::create_request);
    ClassDB::bind_method(D_METHOD("async_send_request", "request"), &RosClient::async_send_request);

    // Emitted on the main thread by wait_for_service_async; await it directly.
    ADD_SIGNAL(MethodInfo("service_available", PropertyInfo(Variant::BOOL, "available")));
}

void RosClient::setup(std::shared_ptr<rclcpp::Node> p_node, const String &p_srv_name, const String &p_srv_type, const Ref<RosQoS> &p_qos)
{
    service_type_ = p_srv_type.utf8().get_data();

    // Ensure the rclgd singleton exists before trying to access BabelFish
    auto rclgd_ptr = rclgd::get_singleton();
    ERR_FAIL_NULL_V_EDMSG(rclgd_ptr, , "RCLGD Singleton is null. Is the extension initialized?");

    rmw_qos_profile_t qos_profile = p_qos.is_valid()
        ? p_qos->get_qos().get_rmw_qos_profile()
        : rmw_qos_profile_services_default;

    try
    {
        client_ = rclgd_ptr->get_fish().create_service_client(
            *p_node,
            p_srv_name.utf8().get_data(),
            p_srv_type.utf8().get_data(),
            qos_profile);
    }
    catch (const std::exception &e)
    {
        RCLGD_FAIL_MSG(vformat("RCLGD: Failed to create service client '%s' (%s): %s",
                               p_srv_name, p_srv_type, e.what()));
    }
}

bool RosClient::wait_for_service(double p_timeout_sec)
{
    // If the client failed to setup, this is a condition we should warn about
    ERR_FAIL_COND_V_EDMSG(!client_, false, "Cannot wait_for_service: Service client is not initialized.");

    return client_->wait_for_service(std::chrono::duration<double>(p_timeout_sec));
}

void RosClient::wait_for_service_async(double p_timeout_sec)
{
    if (!client_)
    {
        // Report failure on the main thread so callers can always `await`.
        call_deferred("emit_signal", "service_available", false);
        return;
    }
    if (wait_thread_.is_valid() && wait_thread_->is_started())
    {
        WARN_PRINT_ED("RosClient: a wait_for_service_async is already in progress.");
        return;
    }

    // Run the blocking wait on a Godot Thread so the main thread (and the ROS
    // executor) keep running. The result is marshalled back to the main thread
    // where the worker is joined and the signal emitted.
    wait_thread_.instantiate();
    wait_thread_->start(callable_mp(this, &RosClient::_run_wait_for_service).bind(p_timeout_sec));
}

void RosClient::_run_wait_for_service(double p_timeout_sec)
{
    bool ok = client_ && client_->wait_for_service(std::chrono::duration<double>(p_timeout_sec));
    // Hop back to the main thread to join the thread and emit the signal.
    callable_mp(this, &RosClient::_finish_wait_for_service).call_deferred(ok);
}

void RosClient::_finish_wait_for_service(bool p_available)
{
    if (wait_thread_.is_valid())
    {
        wait_thread_->wait_to_finish(); // worker has already returned; this just joins
        wait_thread_.unref();
    }
    emit_signal("service_available", p_available);
}

RosClient::~RosClient()
{
    // If a wait is still in flight when the client is dropped, join it so the
    // Godot Thread is cleaned up (bounded by the wait's timeout).
    if (wait_thread_.is_valid() && wait_thread_->is_started())
        wait_thread_->wait_to_finish();
}

Ref<RosMsg> RosClient::create_request()
{
    // Check if setup was actually successful
    ERR_FAIL_COND_V_EDMSG(service_type_.empty(), nullptr, "Cannot create_request: Service type is unknown. Did setup() fail?");

    auto rclgd_ptr = rclgd::get_singleton();
    ERR_FAIL_NULL_V_EDMSG(rclgd_ptr, nullptr, "RCLGD Singleton vanished during request creation.");

    try
    {
        auto req_msg = rclgd_ptr->get_fish().create_service_request_shared(service_type_);
        Ref<RosMsg> req;
        req.instantiate();
        req->init_babel(req_msg);
        return req;
    }
    catch (const std::exception &e)
    {
        RCLGD_FAIL_COND_V_MSG(true, godot::Ref<RosMsg>(), vformat("BabelFish failed to create request for type '%s': %s", String(service_type_.c_str()), e.what()));
    }
}

Ref<RosRequest> RosClient::async_send_request(Ref<RosMsg> p_req) {
    RCLGD_FAIL_COND_V_MSG(!client_, nullptr, "RosClient: Service client not initialized.");
    RCLGD_FAIL_COND_V_MSG(!p_req.is_valid(), nullptr, "RosClient: Invalid RosMsg provided.");

    // Create the "Future" object
    Ref<RosRequest> ros_req;
    ros_req.instantiate();

    // The lambda captures ros_req by value, incrementing the RefCount, callback is in charge of setting the response and emitting the signal
    client_->async_send_request(p_req->get_babel(),
        [ros_req](std::shared_future<ros_babel_fish::CompoundMessage::SharedPtr> future) {

            // 1. Get result from ROS thread
            auto response_ptr = future.get();

            // 2. Prepare the Godot message
            Ref<RosMsg> res_msg;
            res_msg.instantiate();
            if (response_ptr) {
                res_msg->init_babel(response_ptr);
            }

            // 3. Defer the whole completion (state + signal) to the main
            // Godot thread so it never races a GDScript poller reading
            // is_completed()/get_response() mid-update.
            ros_req->call_deferred("_complete", res_msg);
        });

    return ros_req;
}