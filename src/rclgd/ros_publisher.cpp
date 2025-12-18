#include "ros_publisher.hpp"
#include <godot_cpp/variant/utility_functions.hpp>

void RosPublisher::_bind_methods() {
    ClassDB::bind_method(D_METHOD("publish", "msg"), &RosPublisher::publish);
}

void RosPublisher::setup(const std::shared_ptr<rclcpp::Node> &node, const String &topic, const String &type) {
    if (!rclgd::get_singleton()) return;

    std::string std_topic = topic.utf8().get_data();

    // Access the global BabelFish instance from the singleton
    auto &fish = rclgd::get_singleton()->get_fish();

    // Create the publisher using the global instance
    pub_ = fish.create_publisher(*node, std_topic, type.utf8().get_data(), 10);
}

void RosPublisher::publish(const Ref<RosMsg> &msg) {
    if (!pub_) {
        UtilityFunctions::printerr("ROS Error: Publisher not initialized.");
        return;
    }

    if (msg.is_null()) {
        UtilityFunctions::printerr("ROS Error: Attempted to publish null message.");
        return;
    }

    // BabelFishPublisher::publish takes a CompoundMessage
    // msg->get_babel_msg() should return the internal Babel Fish message pointer
    pub_->publish(*msg->get_msg());
}