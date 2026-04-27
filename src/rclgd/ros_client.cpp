#include "ros_client.hpp"
#include "rclgd.hpp"

void RosClient::_bind_methods() {
    ClassDB::bind_method(D_METHOD("wait_for_service", "timeout_sec"), &RosClient::wait_for_service);
    ClassDB::bind_method(D_METHOD("create_request"), &RosClient::create_request);
    ClassDB::bind_method(D_METHOD("async_send_request", "request"), &RosClient::async_send_request);
}

void RosClient::setup(std::shared_ptr<rclcpp::Node> p_node, const String &p_srv_name, const String &p_srv_type)
{
    service_type_ = p_srv_type.utf8().get_data();

    // Ensure the rclgd singleton exists before trying to access BabelFish
    auto rclgd_ptr = rclgd::get_singleton();
    ERR_FAIL_NULL_V_EDMSG(rclgd_ptr, , "RCLGD Singleton is null. Is the extension initialized?");

    try
    {
        client_ = rclgd_ptr->get_fish().create_service_client(
            *p_node,
            p_srv_name.utf8().get_data(),
            p_srv_type.utf8().get_data());
    }
    catch (const std::exception &e)
    {
        RCLGD_FAIL_MSG(vformat("BabelFish failed to create request..."));
    }
}

bool RosClient::wait_for_service(double p_timeout_sec)
{
    // If the client failed to setup, this is a condition we should warn about
    ERR_FAIL_COND_V_EDMSG(!client_, false, "Cannot wait_for_service: Service client is not initialized.");

    return client_->wait_for_service(std::chrono::duration<double>(p_timeout_sec));
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

            // 3. Defer the signal emission to the main Godot thread
            // This prevents race conditions with GDScript execution
            ros_req->_set_finished(res_msg);
        });

    return ros_req;
}