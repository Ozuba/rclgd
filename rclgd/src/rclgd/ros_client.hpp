#ifndef ROS_CLIENT_HPP
#define ROS_CLIENT_HPP

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/classes/thread.hpp>
#include <ros_babel_fish/babel_fish.hpp>
#include "ros_msg.hpp"
#include "ros_qos.hpp"

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
        // Bound so the executor thread can reach it through call_deferred
        ClassDB::bind_method(D_METHOD("_complete", "response"), &RosRequest::_complete);

        // This is the signal GDScript will 'await'
        ADD_SIGNAL(MethodInfo("completed", PropertyInfo(Variant::OBJECT, "response", PROPERTY_HINT_RESOURCE_TYPE, "RosMsg")));
    }

public:
    Ref<RosMsg> get_response() { return response; }
    bool is_completed() { return completed; }

    // Internal use only to fulfill the "Promise". Runs on the main thread
    // (dispatched via call_deferred from the executor thread) so the state
    // mutation and the signal emission never race a GDScript poller.
    void _complete(Ref<RosMsg> p_res) {
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

    // Background Godot Thread backing wait_for_service_async.
    Ref<Thread> wait_thread_;
    void _run_wait_for_service(double p_timeout_sec); // thread body
    void _finish_wait_for_service(bool p_available);  // main thread: join + emit

protected:
    static void _bind_methods();

public:
    RosClient() = default;
    ~RosClient();
    void setup(std::shared_ptr<rclcpp::Node> p_node, const String &p_srv_name, const String &p_srv_type, const Ref<RosQoS> &p_qos = Ref<RosQoS>());

    // Blocking: stalls the calling thread for up to p_timeout_sec.
    bool wait_for_service(double p_timeout_sec);
    // Non-blocking: returns immediately and emits "service_available(bool)" on
    // the main thread once the service appears or the timeout elapses. Use as
    // `client.wait_for_service_async(t); var ok = await client.service_available`.
    void wait_for_service_async(double p_timeout_sec);

    Ref<RosMsg> create_request();

    // Returns the RosRequest handle
    Ref<RosRequest> async_send_request(Ref<RosMsg> p_req);
};

#endif