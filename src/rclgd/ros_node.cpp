#include "ros_node.hpp"
#include "rclgd.hpp"
#include <godot_cpp/variant/utility_functions.hpp>

void RosNode::_bind_methods()
{
    //Initialization
    ClassDB::bind_method(D_METHOD("init", "node_name"), &RosNode::init);

    //Factory Methods
    ClassDB::bind_method(D_METHOD("create_publisher", "topic", "type"), &RosNode::create_publisher);
    ClassDB::bind_method(D_METHOD("create_subscriber", "topic", "type", "callback"), &RosNode::create_subscriber);
    ClassDB::bind_method(D_METHOD("create_client", "srv_name", "srv_type"), &RosNode::create_client);
    ClassDB::bind_method(D_METHOD("create_service", "srv_name", "srv_type", "callback"), &RosNode::create_service);

    //Functions
    ClassDB::bind_method(D_METHOD("now"), &RosNode::now);

    // Parameters
    ClassDB::bind_method(D_METHOD("declare_parameter", "name", "default_value"), &RosNode::declare_parameter);
    ClassDB::bind_method(D_METHOD("set_parameter", "name", "value"), &RosNode::set_parameter);
    ClassDB::bind_method(D_METHOD("get_parameter", "name"), &RosNode::get_parameter);

    // Signals
    ADD_SIGNAL(MethodInfo("parameter_changed", PropertyInfo(Variant::STRING, "name"), PropertyInfo(Variant::NIL, "value")));
}

void RosNode::init(const String &p_node_name)
{
    if (node_)
        return; // Prevent double initialization

    std::string std_name = p_node_name.utf8().get_data();
    node_ = std::make_shared<rclcpp::Node>(std_name);

    if (rclgd::get_singleton())
    {
        rclgd::get_singleton()->add_node(node_);
        UtilityFunctions::print("Standalone ROS Node initialized: ", p_node_name);
    }
    
    //Parameter update
    param_callback_handle_ = node_->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &parameters) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            for (const auto &param : parameters) {
                Variant val = RosTypeMapping::ros_param_to_variant(param);
                // Move to Godot thread
                this->call_deferred("emit_signal", "parameter_changed", String(param.get_name().c_str()), val);
            }
            return result;
        }
    );
}

RosNode::~RosNode()
{
    // When the GDScript variable is freed, we should remove the node from the ROS executor
    if (node_ && rclgd::get_singleton())
    {
        rclgd::get_singleton()->remove_node(node_);
    }
}

void RosNode::declare_parameter(const String &p_name, const Variant &p_default_value) {
    if (!node_) return;
    
    std::string name = p_name.utf8().get_data();
    
    // Use your logic to convert the Godot default value into a ROS parameter value
    auto ros_value = RosTypeMapping::variant_to_ros_param(p_default_value);
    
    // rclcpp call
    node_->declare_parameter(name, ros_value);
}

void RosNode::set_parameter(const String &p_name, const Variant &p_val) {
    std::string name = p_name.utf8().get_data();
    
    // Using your conversion logic
    auto ros_val = RosTypeMapping::variant_to_ros_param(p_val);
    node_->set_parameter(rclcpp::Parameter(name, ros_val));
}

Variant RosNode::get_parameter(const String &p_name) {
    std::string name = p_name.utf8().get_data();
    if (!node_->has_parameter(name)) return Variant();
    
    auto param = node_->get_parameter(name);
    return RosTypeMapping::ros_param_to_variant(param);
}

Ref<RosPublisher> RosNode::create_publisher(const String &topic, const String &type)
{
    ERR_FAIL_COND_V_MSG(!rclcpp::ok(), nullptr, "ROS2 Global Context is not OK. Did it shut down?");
    ERR_FAIL_COND_V_MSG(!node_, nullptr, "RosNode must be initialized before creating publishers.");

    Ref<RosPublisher> pub;
    pub.instantiate();
    pub->setup(node_, topic, type);
    return pub;
}

Ref<RosSubscriber> RosNode::create_subscriber(const String &topic, const String &type, const Callable &callback)
{
    ERR_FAIL_COND_V_MSG(!rclcpp::ok(), nullptr, "ROS2 Global Context is not OK. Did it shut down?");
    ERR_FAIL_COND_V_MSG(!node_, nullptr, "RosNode must be initialized before creating subscribers.");

    Ref<RosSubscriber> sub;
    sub.instantiate();
    sub->setup(node_, topic, type, callback);
    return sub;
}

Ref<RosClient> RosNode::create_client(const String &p_srv_name, const String &p_srv_type)
{
    ERR_FAIL_COND_V_MSG(!rclcpp::ok(), nullptr, "ROS2 Global Context is not OK. Did it shut down?");
    ERR_FAIL_COND_V_MSG(!node_, nullptr, "RosNode must be initialized before creating clients.");

    Ref<RosClient> client;
    client.instantiate();
    // The factory logic happens here
    client->setup(node_, p_srv_name, p_srv_type);
    return client;
}


Ref<RosService> RosNode::create_service(const String &p_srv_name, const String &p_srv_type, const Callable &p_callback) {
    ERR_FAIL_COND_V_MSG(!rclcpp::ok(), nullptr, "ROS2 Global Context is not OK. Did it shut down?");
    ERR_FAIL_COND_V_MSG(!node_, nullptr, "RosNode must be initialized before creating services.");

    if (p_callback.is_null()) {
        UtilityFunctions::push_error("ROS2: Cannot create service with a null callback.");
        return nullptr;
    }

    Ref<RosService> srv;
    srv.instantiate();
    srv->setup(node_, p_srv_name, p_srv_type, p_callback);
    return srv;
}

Ref<RosMsg> RosNode::now()
{
    // 1. Create the specific ROS Time message
    Ref<RosMsg> time_msg;
    time_msg.instantiate();
    // Assuming you have a way to initialize the internal BabelFish message by type
    // This should match your "RosMsg::from_type" logic
    time_msg->from_type("builtin_interfaces/msg/Time");

    if (node_)
    {
        rclcpp::Time now = node_->now();

        // 2. Set the values using your existing _set logic
        // (or direct BabelFish access for speed)
        time_msg->set("sec", (int32_t)now.seconds());
        time_msg->set("nanosec", (uint32_t)(now.nanoseconds() % 1000000000));
    }

    return time_msg;
}