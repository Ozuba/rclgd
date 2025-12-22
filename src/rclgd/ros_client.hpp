#ifndef ROS_CLIENT_HPP
#define ROS_CLIENT_HPP

#include <godot_cpp/classes/ref_counted.hpp>
#include <ros_babel_fish/babel_fish.hpp>
#include "ros_msg.hpp"

using namespace godot;

class RosClient : public RefCounted {
    GDCLASS(RosClient, RefCounted);

private:
    ros_babel_fish::BabelFishServiceClient::SharedPtr client_;
    std::string service_type_;
protected:
    static void _bind_methods();

public:
    RosClient() = default;
    
    // Internal init called by the RosNode factory
    void setup(std::shared_ptr<rclcpp::Node> p_node, const String &p_srv_name, const String &p_srv_type);

    bool wait_for_service(double p_timeout_sec);
    Ref<RosMsg> create_request();
    void async_send_request(Ref<RosMsg> p_req);
};

#endif