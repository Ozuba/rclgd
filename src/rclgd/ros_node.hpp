#pragma once

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/script.hpp>

#include <rclcpp/rclcpp.hpp>

#include "ros_publisher.hpp"
#include "ros_subscriber.hpp"
#include "ros_client.hpp"
#include "ros_service.hpp"
#include "ros_tf_broadcaster.hpp"
#include "ros_tf_listener.hpp"
#include "ros_timer.hpp"
#include "ros_msg.hpp"

#include "utils/ros_type_utils.hpp"
#include "utils/ros_tf_utils.hpp"

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
    void init(const String &p_node_name, const String &p_namespace);

    //Namespace and name getters
    String get_name() const;
    String get_namespace() const;

    // Helper to get ros types from shadow scripts
    String _get_type_from_variant(const Variant &p_type);

    // Publisher and subscription creation
    Ref<RosPublisher> create_publisher(const String &topic, const Variant &type, const Ref<RosQoS> &qos = Ref<RosQoS>());
    Ref<RosSubscriber> create_subscription(const String &topic, const Variant &type, const Callable &callback, const Ref<RosQoS> &qos= Ref<RosQoS>());

    // Service client and subscriber
    Ref<RosClient> create_client(const String &p_srv_name, const Variant &p_srv_type);
    Ref<RosService> create_service(const String &p_srv_name, const Variant &p_srv_type, const Callable &p_callback);

    // Timers
    Ref<RosTimer> create_timer(double p_seconds, const Callable &p_callback);

    //TF2 support built in
    Ref<RosTfListener> create_tf_listener();
    Ref<RosTfBroadcaster> create_tf_broadcaster();
    String resolve_frame(const String &p_id);

    // Time related
    Ref<RosMsg> now();

    //Parameters
    void declare_parameter(const String &p_name, const Variant &p_default_value);
    void set_parameter(const String &p_name, const Variant &p_val);
    Variant get_parameter(const String &p_name);

    //Ros graph Inspection
    Dictionary get_topic_names_and_types();
    int count_publishers(const String &p_topic);
    int count_subscribers(const String &p_topic);
};