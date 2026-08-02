#include "ros_node.hpp"
#include "rclgd.hpp"
#include <godot_cpp/variant/utility_functions.hpp>

void RosNode::_bind_methods()
{
    // Initialization
    ClassDB::bind_method(D_METHOD("init", "node_name", "namespace"), &RosNode::init, DEFVAL(""));

    // Getters for internal values
    ClassDB::bind_method(D_METHOD("get_name"), &RosNode::get_name);
    ClassDB::bind_method(D_METHOD("get_namespace"), &RosNode::get_namespace);

    // Factory Methods
    ClassDB::bind_method(D_METHOD("create_publisher", "topic", "type", "qos"),
                         &RosNode::create_publisher,
                         DEFVAL(Ref<RosQoS>()));

    ClassDB::bind_method(D_METHOD("create_subscription", "topic", "type", "callback", "qos"),
                         &RosNode::create_subscription,
                         DEFVAL(Ref<RosQoS>()));

    ClassDB::bind_method(D_METHOD("create_client", "srv_name", "srv_type", "qos"),
                         &RosNode::create_client,
                         DEFVAL(Ref<RosQoS>()));

    ClassDB::bind_method(D_METHOD("create_service", "srv_name", "srv_type", "callback", "qos"),
                         &RosNode::create_service,
                         DEFVAL(Ref<RosQoS>()));

    ClassDB::bind_method(D_METHOD("create_action_client", "action_name", "action_type", "qos"),
                         &RosNode::create_action_client,
                         DEFVAL(Ref<RosQoS>()));

    ClassDB::bind_method(D_METHOD("create_action_server", "action_name", "action_type", "execute_callback", "goal_callback", "qos"),
                         &RosNode::create_action_server,
                         DEFVAL(Callable()), DEFVAL(Ref<RosQoS>()));
    ClassDB::bind_method(D_METHOD("create_tf_broadcaster"), &RosNode::create_tf_broadcaster);
    ClassDB::bind_method(D_METHOD("create_tf_listener", "cache_time_sec"), &RosNode::create_tf_listener,
                         DEFVAL(10.0));
    ClassDB::bind_method(D_METHOD("create_timer", "seconds", "callback"), &RosNode::create_timer);

    // Functions
    ClassDB::bind_method(D_METHOD("now"), &RosNode::now);

    // Parameters
    ClassDB::bind_method(D_METHOD("declare_parameter", "name", "default_value"), &RosNode::declare_parameter);
    ClassDB::bind_method(D_METHOD("set_parameter", "name", "value"), &RosNode::set_parameter);
    ClassDB::bind_method(D_METHOD("get_parameter", "name"), &RosNode::get_parameter);

    // ROS Graph Introspection
    ClassDB::bind_method(D_METHOD("get_topic_names_and_types"), &RosNode::get_topic_names_and_types);
    ClassDB::bind_method(D_METHOD("count_publishers", "topic"), &RosNode::count_publishers);
    ClassDB::bind_method(D_METHOD("count_subscribers", "topic"), &RosNode::count_subscribers);

    //Helpers
    ClassDB::bind_method(D_METHOD("resolve_frame", "frame_id"), &RosNode::resolve_frame);

    // Signals
    ADD_SIGNAL(MethodInfo("parameter_changed", PropertyInfo(Variant::STRING, "name"), PropertyInfo(Variant::NIL, "value")));
}

void RosNode::init(const String &p_node_name, const String &p_namespace)
{
    if (node_)
    {
        WARN_PRINT_ED("RCLGD: RosNode is already initialized. Ignoring second init call.");
        return;
    }

    // Convert Godot Strings to std::string
    std::string std_name = p_node_name.utf8().get_data();
    std::string std_ns = p_namespace.utf8().get_data();

    // ROS 2 node names cannot be empty.
    // Namespaces can be empty (defaults to global '/')
    if (std_name.empty())
    {
        RCLGD_FAIL_MSG("RCLGD: Cannot initialize RosNode with an empty name.");
        return;
    }

    try
    {
        /* Initialize the rclcpp::Node.
           Note: rclcpp will throw an exception if:
           1. node_name contains slashes.
           2. namespace is invalid.
           3. node_name contains illegal characters.
        */
        node_ = std::make_shared<rclcpp::Node>(std_name, std_ns);
    }
    catch (const std::exception &e)
    {
        RCLGD_FAIL_MSG(vformat("RCLGD: Failed to create rclcpp::Node '%s' in namespace '%s'. Reason: %s",
                               p_node_name, p_namespace, e.what()));
        return;
    }

    // Register node with the global executor managed by the rclgd singleton
    if (rclgd::get_singleton())
    {
        rclgd::get_singleton()->add_node(node_);
    }
    else
    {
        RCLGD_FAIL_MSG("RCLGD: Global singleton not found. Node will not be processed by the executor.");
    }

    // Setup Parameter update callback
    param_callback_handle_ = node_->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &parameters)
        {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            for (const auto &param : parameters)
            {
                // Assuming RosTypeMapping exists as per your snippet
                Variant val = RosTypeMapping::ros_param_to_variant(param);

                // Emit signal on Godot thread to remain thread-safe with the UI/Editor
                this->call_deferred("emit_signal", "parameter_changed", String(param.get_name().c_str()), val);
            }
            return result;
        });

    UtilityFunctions::print_verbose(vformat("RCLGD: Node '%s' initialized in namespace '%s'", p_node_name, p_namespace));
}

String RosNode::get_name() const
{
    if (!node_)
        return String();
    return String(node_->get_name());
}

String RosNode::get_namespace() const
{
    if (!node_)
        return String();

    return String(node_->get_namespace());
}

RosNode::~RosNode()
{
    // When the GDScript variable is freed, we should remove the node from the ROS executor.
    // The singleton may already be gone during engine teardown, so check it.
    if (node_ && rclgd::get_singleton())
    {
        rclgd::get_singleton()->remove_node(node_);
    }
}

void RosNode::declare_parameter(const String &p_name, const Variant &p_default_value)
{
    if (!node_)
        return;

    std::string name = p_name.utf8().get_data();

    // Use your logic to convert the Godot default value into a ROS parameter value
    auto ros_value = RosTypeMapping::variant_to_ros_param(p_default_value);

    // rclcpp throws if the parameter is already declared (or a constraint
    // rejects the value); funnel it through the RCLGD error path instead of
    // letting the exception escape into the engine.
    try
    {
        node_->declare_parameter(name, ros_value);
    }
    catch (const std::exception &e)
    {
        RCLGD_FAIL_MSG(vformat("RCLGD: Failed to declare parameter '%s': %s", p_name, e.what()));
    }
}

void RosNode::set_parameter(const String &p_name, const Variant &p_val)
{
    RCLGD_FAIL_COND_MSG(!node_, "RosNode must be initialized before setting parameters.");

    std::string name = p_name.utf8().get_data();
    auto ros_val = RosTypeMapping::variant_to_ros_param(p_val);
    // set_parameter throws if the parameter was never declared (unless the node
    // allows undeclared params) or a set-callback rejects it.
    try
    {
        node_->set_parameter(rclcpp::Parameter(name, ros_val));
    }
    catch (const std::exception &e)
    {
        RCLGD_FAIL_MSG(vformat("RCLGD: Failed to set parameter '%s': %s", p_name, e.what()));
    }
}

Variant RosNode::get_parameter(const String &p_name)
{
    RCLGD_FAIL_COND_V_MSG(!node_, Variant(), "RosNode must be initialized before getting parameters.");

    std::string name = p_name.utf8().get_data();
    if (!node_->has_parameter(name))
        return Variant();

    auto param = node_->get_parameter(name);
    return RosTypeMapping::ros_param_to_variant(param);
}


String RosNode::_get_type_from_variant(const Variant &p_type) {
    // Handle raw Strings (e.g., "std_msgs/msg/String")
    if (p_type.get_type() == Variant::STRING) {
        return p_type;
    }

    // Handle Objects (Scripts or Instances)
    if (p_type.get_type() == Variant::OBJECT) {
        Ref<Script> scr = p_type;
        if (scr.is_valid()) {
            Dictionary constants = scr->get_script_constant_map();
            if (constants.has("ROS_TYPE_NAME")) {
                return constants["ROS_TYPE_NAME"];
            }
        }
    }
    return ""; // Failed to resolve
}

Ref<RosPublisher> RosNode::create_publisher(const String &topic, const Variant &type, const Ref<RosQoS> &qos)
{
    RCLGD_FAIL_COND_V_MSG(!node_, nullptr, "RosNode must be initialized.");
    String ros_type = _get_type_from_variant(type);
    RCLGD_FAIL_COND_V_MSG(ros_type.is_empty(), nullptr, "RCLGD: Could not resolve ROS type for Publisher.");
    Ref<RosPublisher> pub;
    pub.instantiate();
    pub->setup(node_, topic, ros_type, qos);
    return pub;
}

Ref<RosSubscriber> RosNode::create_subscription(const String &topic, const Variant &type, const Callable &callback, const Ref<RosQoS> &qos)
{
    RCLGD_FAIL_COND_V_MSG(!node_, nullptr, "RosNode must be initialized.");
    String ros_type = _get_type_from_variant(type);
    RCLGD_FAIL_COND_V_MSG(ros_type.is_empty(), nullptr, "RCLGD: Could not resolve ROS type for Subscriber.");
    Ref<RosSubscriber> sub;
    sub.instantiate();
    sub->setup(node_, topic, ros_type, callback, qos);
    return sub;
}

Ref<RosClient> RosNode::create_client(const String &p_srv_name, const Variant &p_srv_type, const Ref<RosQoS> &qos)
{
    RCLGD_FAIL_COND_V_MSG(!node_, nullptr, "RosNode must be initialized.");
    String ros_type = _get_type_from_variant(p_srv_type);
    RCLGD_FAIL_COND_V_MSG(ros_type.is_empty(), nullptr, "RCLGD: Could not resolve ROS type for Client.");
    Ref<RosClient> client;
    client.instantiate();
    client->setup(node_, p_srv_name, ros_type, qos);
    return client;
}

Ref<RosService> RosNode::create_service(const String &p_srv_name, const Variant &p_srv_type, const Callable &p_callback, const Ref<RosQoS> &qos)
{
    RCLGD_FAIL_COND_V_MSG(!node_, nullptr, "RosNode must be initialized.");
    String ros_type = _get_type_from_variant(p_srv_type);
    RCLGD_FAIL_COND_V_MSG(ros_type.is_empty(), nullptr, "RCLGD: Could not resolve ROS type for Service.");
    Ref<RosService> srv;
    srv.instantiate();
    srv->setup(node_, p_srv_name, ros_type, p_callback, qos);
    return srv;
}

Ref<RosActionClient> RosNode::create_action_client(const String &p_action_name, const Variant &p_action_type, const Ref<RosQoS> &qos)
{
    RCLGD_FAIL_COND_V_MSG(!node_, nullptr, "RosNode must be initialized.");
    String ros_type = _get_type_from_variant(p_action_type);
    RCLGD_FAIL_COND_V_MSG(ros_type.is_empty(), nullptr, "RCLGD: Could not resolve ROS type for Action Client.");
    Ref<RosActionClient> client;
    client.instantiate();
    client->setup(node_, p_action_name, ros_type, qos);
    return client;
}

Ref<RosActionServer> RosNode::create_action_server(const String &p_action_name, const Variant &p_action_type, const Callable &p_execute_callback, const Callable &p_goal_callback, const Ref<RosQoS> &qos)
{
    RCLGD_FAIL_COND_V_MSG(!node_, nullptr, "RosNode must be initialized.");
    String ros_type = _get_type_from_variant(p_action_type);
    RCLGD_FAIL_COND_V_MSG(ros_type.is_empty(), nullptr, "RCLGD: Could not resolve ROS type for Action Server.");
    Ref<RosActionServer> srv;
    srv.instantiate();
    srv->setup(node_, p_action_name, ros_type, p_execute_callback, p_goal_callback, qos);
    return srv;
}

Ref<RosTfBroadcaster> RosNode::create_tf_broadcaster()
{
    RCLGD_FAIL_COND_V_MSG(!rclcpp::ok(), nullptr, "ROS2 Global Context is not OK. Did it shut down?");
    RCLGD_FAIL_COND_V_MSG(!node_, nullptr, "RosNode must be initialized before creating TF Broadcasters.");
    Ref<RosTfBroadcaster> bc;
    bc.instantiate();
    bc->setup(node_);
    return bc;
}

Ref<RosTfListener> RosNode::create_tf_listener(double p_cache_time_sec)
{
    RCLGD_FAIL_COND_V_MSG(!rclcpp::ok(), nullptr, "ROS2 Global Context is not OK. Did it shut down?");
    RCLGD_FAIL_COND_V_MSG(!node_, nullptr, "RosNode must be initialized before creating TF Listeners.");
    Ref<RosTfListener> ls;
    ls.instantiate();
    ls->setup(node_, p_cache_time_sec);
    return ls;
}

Ref<RosTimer> RosNode::create_timer(double p_seconds, const Callable &p_callback) {
    RCLGD_FAIL_COND_V_MSG(!node_, nullptr, "RosNode must be initialized before creating timers.");
    Ref<RosTimer> timer;
    timer.instantiate();
    timer->setup(node_, p_seconds, p_callback);
    return timer;
}

//Helper to resolve namespaced topics (Helps creating multisensor simulations with ease)
String RosNode::resolve_frame(const String &p_id) {
    return String(RclgdUtils::resolve_frame(node_, p_id).c_str());
}


Ref<RosMsg> RosNode::now()
{
    // 1. Create the specific ROS Time message
    Ref<RosMsg> time_msg;
    time_msg.instantiate();
    time_msg->init("builtin_interfaces/msg/Time");

    if (node_)
    {
        // Derive both fields from the integer nanosecond count to avoid any
        // floating point rounding mismatch at second boundaries.
        int64_t ns = node_->now().nanoseconds();
        time_msg->set("sec", (int64_t)(ns / 1000000000));
        time_msg->set("nanosec", (int64_t)(ns % 1000000000));
    }

    return time_msg;
}

Dictionary RosNode::get_topic_names_and_types()
{
    Dictionary res;
    RCLGD_FAIL_COND_V_MSG(!node_, res, "RosNode not initialized.");

    // Get the map from rclcpp
    auto topics = node_->get_topic_names_and_types();

    for (const auto &it : topics)
    {
        String topic_name = String(it.first.c_str());
        Array types;
        for (const auto &type_str : it.second)
        {
            types.push_back(String(type_str.c_str()));
        }
        res[topic_name] = types;
    }

    return res;
}

int RosNode::count_publishers(const String &p_topic)
{
    ERR_FAIL_COND_V(!node_, 0);
    std::string topic = p_topic.utf8().get_data();
    return static_cast<int>(node_->count_publishers(topic));
}

int RosNode::count_subscribers(const String &p_topic)
{
    ERR_FAIL_COND_V(!node_, 0);
    std::string topic = p_topic.utf8().get_data();
    return static_cast<int>(node_->count_subscribers(topic));
}