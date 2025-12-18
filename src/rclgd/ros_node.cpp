#include "ros_node.hpp"
#include "rclgd.hpp"
#include <godot_cpp/variant/utility_functions.hpp>

void RosNode::_bind_methods() {
    ClassDB::bind_method(D_METHOD("init", "node_name"), &RosNode::init);
    ClassDB::bind_method(D_METHOD("create_publisher", "topic", "type"), &RosNode::create_publisher);
    ClassDB::bind_method(D_METHOD("create_subscriber", "topic", "type", "callback"), &RosNode::create_subscriber);
    ClassDB::bind_method(D_METHOD("now"), &RosNode::now);
}

void RosNode::init(const String &p_node_name) {
    if (node_) return; // Prevent double initialization

    std::string std_name = p_node_name.utf8().get_data();
    node_ = std::make_shared<rclcpp::Node>(std_name);

    if (rclgd::get_singleton()) {
        rclgd::get_singleton()->add_node(node_);
        UtilityFunctions::print("Standalone ROS Node initialized: ", p_node_name);
    }
}

RosNode::~RosNode() {
    // When the GDScript variable is freed, we should remove the node from the ROS executor
    if (node_ && rclgd::get_singleton()) {
        rclgd::get_singleton()->remove_node(node_);
    }
}

Ref<RosPublisher> RosNode::create_publisher(const String &topic, const String &type) {
    ERR_FAIL_COND_V_MSG(!node_, nullptr, "RosNode must be initialized before creating publishers.");
    
    Ref<RosPublisher> pub;
    pub.instantiate();
    pub->setup(node_, topic, type);
    return pub;
}

Ref<RosSubscriber> RosNode::create_subscriber(const String &topic, const String &type, const Callable &callback) {
    ERR_FAIL_COND_V_MSG(!node_, nullptr, "RosNode must be initialized before creating subscribers.");

    Ref<RosSubscriber> sub;
    sub.instantiate();
    sub->setup(node_, topic, type, callback);
    return sub;
}



Ref<RosMsg> RosNode::now() {
    // 1. Create the specific ROS Time message
    Ref<RosMsg> time_msg;
    time_msg.instantiate();
    // Assuming you have a way to initialize the internal BabelFish message by type
    // This should match your "RosMsg::from_type" logic
    time_msg->from_type("builtin_interfaces/msg/Time");

    if (node_) {
        rclcpp::Time now = node_->now();
        
        // 2. Set the values using your existing _set logic 
        // (or direct BabelFish access for speed)
        time_msg->set("sec", (int32_t)now.seconds());
        time_msg->set("nanosec", (uint32_t)(now.nanoseconds() % 1000000000));
    }

    return time_msg;
}