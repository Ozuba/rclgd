#include "ros_publisher.hpp"
#include <godot_cpp/variant/utility_functions.hpp>

void RosPublisher::_bind_methods() {
    ClassDB::bind_method(D_METHOD("publish", "msg"), &RosPublisher::publish);
}

void RosPublisher::setup(const std::shared_ptr<rclcpp::Node> &node, const String &topic, const String &type, const Ref<RosQoS> &qos) {
    if (!rclgd::get_singleton()) return;

    std::string std_topic = topic.utf8().get_data();
    // No RosQoS given -> the ROS default topic profile (keep-last 10)
    rclcpp::QoS final_qos = qos.is_valid() ? qos->get_qos() : rclcpp::QoS(10);

    // Access the global BabelFish instance from the singleton
    auto &fish = rclgd::get_singleton()->get_fish();

    // Create the publisher using the global instance
    try{
    pub_ = fish.create_publisher(*node, std_topic, type.utf8().get_data(), final_qos);
    }catch (const std::exception &e)
    {
        RCLGD_FAIL_MSG(vformat("RCLGD Publisher failed: %s", e.what()));
    }
}

void RosPublisher::publish(const Ref<RosMsg> &msg) {

    if (!pub_) {
        RCLGD_FAIL_MSG("ROS Error: Publisher not initialized.");
        return;
    }

    if (msg.is_null()) {
        RCLGD_FAIL_MSG("ROS Error: Attempted to publish null message.");
        return;
    }

    
    // BabelFishPublisher::publish takes a CompoundMessage
    // msg->get_babel_msg() should return the internal Babel Fish message pointer
    pub_->publish(*msg->get_babel());
}