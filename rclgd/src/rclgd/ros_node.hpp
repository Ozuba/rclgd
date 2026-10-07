#pragma once

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/script.hpp>

#include <rclcpp/rclcpp.hpp>

#include "ros_publisher.hpp"
#include "ros_subscriber.hpp"
#include "ros_client.hpp"
#include "ros_service.hpp"
#include "ros_action_client.hpp"
#include "ros_action_server.hpp"
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

    void _log(int p_severity, const String &p_msg) const;

protected:
    static void _bind_methods();

public:
    // Values mirror RCUTILS_LOG_SEVERITY
    enum LogLevel
    {
        LOG_UNSET = 0,
        LOG_DEBUG = 10,
        LOG_INFO = 20,
        LOG_WARN = 30,
        LOG_ERROR = 40,
        LOG_FATAL = 50,
    };

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

    // Service client and server
    Ref<RosClient> create_client(const String &p_srv_name, const Variant &p_srv_type, const Ref<RosQoS> &qos = Ref<RosQoS>());
    Ref<RosService> create_service(const String &p_srv_name, const Variant &p_srv_type, const Callable &p_callback, const Ref<RosQoS> &qos = Ref<RosQoS>());

    // Action client and server
    Ref<RosActionClient> create_action_client(const String &p_action_name, const Variant &p_action_type, const Ref<RosQoS> &qos = Ref<RosQoS>());
    Ref<RosActionServer> create_action_server(const String &p_action_name, const Variant &p_action_type, const Callable &p_execute_callback, const Callable &p_goal_callback = Callable(), const Ref<RosQoS> &qos = Ref<RosQoS>());

    // Timers
    Ref<RosTimer> create_timer(double p_seconds, const Callable &p_callback);

    //TF2 support built in
    Ref<RosTfListener> create_tf_listener(double p_cache_time_sec = 10.0);
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

    // Logging through the node's rclcpp logger (console + /rosout)
    void log_debug(const String &p_msg) const { _log(LOG_DEBUG, p_msg); }
    void log_info(const String &p_msg) const { _log(LOG_INFO, p_msg); }
    void log_warn(const String &p_msg) const { _log(LOG_WARN, p_msg); }
    void log_error(const String &p_msg) const { _log(LOG_ERROR, p_msg); }
    void log_fatal(const String &p_msg) const { _log(LOG_FATAL, p_msg); }
    void set_log_level(LogLevel p_level);
    LogLevel get_log_level() const;
};

VARIANT_ENUM_CAST(RosNode::LogLevel);