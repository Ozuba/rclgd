#ifndef ROS_CLIENT_HPP
#define ROS_CLIENT_HPP

#include <godot_cpp/classes/ref_counted.hpp>
#include <ros_babel_fish/babel_fish.hpp>
#include "ros_msg.hpp"

using namespace godot;

//Request object
class RosRequest : public RefCounted {
    GDCLASS(RosRequest, RefCounted);

private:
    Ref<RosMsg> response;
    bool completed = false;

protected:
    static void _bind_methods() {
        ClassDB::bind_method(D_METHOD("get_response"), &RosRequest::get_response);
        ClassDB::bind_method(D_METHOD("is_completed"), &RosRequest::is_completed);
        
        // This is the signal GDScript will 'await'
        ADD_SIGNAL(MethodInfo("completed", PropertyInfo(Variant::OBJECT, "response", PROPERTY_HINT_RESOURCE_TYPE, "RosMsg")));
    }

public:
    Ref<RosMsg> get_response() { return response; }
    bool is_completed() { return completed; }
    
    // Internal use only to fulfill the "Promise"
    void _set_finished(Ref<RosMsg> p_res) {
        response = p_res;
        completed = true;
        emit_signal("completed", response);
    }
};

// Client object
class RosClient : public RefCounted {
    GDCLASS(RosClient, RefCounted);

private:
    ros_babel_fish::BabelFishServiceClient::SharedPtr client_;
    std::string service_type_;

protected:
    static void _bind_methods();

public:
    RosClient() = default;
    void setup(std::shared_ptr<rclcpp::Node> p_node, const String &p_srv_name, const String &p_srv_type);
    bool wait_for_service(double p_timeout_sec);
    Ref<RosMsg> create_request();
    
    // Returns the RosRequest handle
    Ref<RosRequest> async_send_request(Ref<RosMsg> p_req);
};

#endif