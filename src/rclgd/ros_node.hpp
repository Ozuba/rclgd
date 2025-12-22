#pragma once

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros_publisher.hpp"
#include "ros_subscriber.hpp"
#include "ros_client.hpp"
#include "ros_service.hpp"
#include "ros_msg.hpp"
#include "utils/ros_type_utils.hpp"

using namespace godot;

class RosNode : public RefCounted
{
    GDCLASS(RosNode, RefCounted)

private:
    std::shared_ptr<rclcpp::Node> node_;

    //Parameter update callback
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

protected:
    static void _bind_methods();

public:
    RosNode() {}
    ~RosNode();

    // Manual initialization since we aren't in the SceneTree
    void init(const String &p_node_name);

    // Publisher and subscriber creation
    Ref<RosPublisher> create_publisher(const String &topic, const String &type);
    Ref<RosSubscriber> create_subscriber(const String &topic, const String &type, const Callable &callback);

    // Service client and subscriber
    Ref<RosClient> create_client(const String &p_srv_name, const String &p_srv_type);
    Ref<RosService> create_service(const String &p_srv_name, const String &p_srv_type, const Callable &p_callback);
    // Time related
    Ref<RosMsg> now();

    //Parameters
    void declare_parameter(const String &p_name, const Variant &p_default_value);
    void set_parameter(const String &p_name, const Variant &p_val);
    Variant get_parameter(const String &p_name);
};