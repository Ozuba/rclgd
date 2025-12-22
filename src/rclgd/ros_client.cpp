#include "ros_client.hpp"
#include "rclgd.hpp" // Access to your BabelFish singleton

void RosClient::_bind_methods() {
    ClassDB::bind_method(D_METHOD("wait_for_service", "timeout_sec"), &RosClient::wait_for_service);
    ClassDB::bind_method(D_METHOD("create_request"), &RosClient::create_request);
    ClassDB::bind_method(D_METHOD("async_send_request", "request"), &RosClient::async_send_request);

    // This signal acts as our "Future"
    ADD_SIGNAL(MethodInfo("request_completed", PropertyInfo(Variant::OBJECT, "response", PROPERTY_HINT_RESOURCE_TYPE, "RosMsg")));
}

void RosClient::setup(std::shared_ptr<rclcpp::Node> p_node, const String &p_srv_name, const String &p_srv_type) {
    service_type_ = p_srv_type.utf8().get_data(); // We do need to store this for create_request()
    
    client_ = rclgd::get_singleton()->get_fish().create_service_client(
        *p_node, 
        p_srv_name.utf8().get_data(), 
        p_srv_type.utf8().get_data()
    );
}

bool RosClient::wait_for_service(double p_timeout_sec) {
    if (!client_) return false;
    return client_->wait_for_service(std::chrono::duration<double>(p_timeout_sec));
}


Ref<RosMsg> RosClient::create_request() {
    if (service_type_.empty()) return nullptr;
    auto req_msg = rclgd::get_singleton()->get_fish().create_service_request_shared(service_type_);
    Ref<RosMsg> req;
    req.instantiate();
    req->init_babel(req_msg); 
    return req;
}

void RosClient::async_send_request(Ref<RosMsg> p_req) {
    if (!client_ || !p_req.is_valid()) return;

    // The lambda must take a shared_future in Jazzy
    client_->async_send_request(p_req->get_babel(), 
        [this](std::shared_future<ros_babel_fish::CompoundMessage::SharedPtr> future) {
            
            // Get the result from the future
            ros_babel_fish::CompoundMessage::SharedPtr response = future.get();

            Ref<RosMsg> res;
            res.instantiate();
            res->init_babel(response);

            // Signal emission back to the main thread
            this->call_deferred("emit_signal", "request_completed", res);
        }
    );
}