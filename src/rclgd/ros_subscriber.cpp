#include "ros_subscriber.hpp"
#include "rclgd.hpp"

void RosSubscriber::setup(const std::shared_ptr<rclcpp::Node> &node, 
                          const String &topic, 
                          const String &type, 
                          const Callable &p_callback) {
    callback_ = p_callback;
    
    std::string std_topic = topic.utf8().get_data();
    std::string std_type = type.utf8().get_data();
    auto &fish = rclgd::get_singleton()->get_fish();

    // Bind our C++ _ros_callback to the BabelFish subscription
    sub_ = fish.create_subscription(*node, std_topic, std_type, 10,
        std::bind(&RosSubscriber::_ros_callback, this, std::placeholders::_1));
}

void RosSubscriber::_ros_callback(const ros_babel_fish::CompoundMessage::SharedPtr msg) {
    if (callback_.is_valid()) {
        // 1. Wrap the raw BabelFish message into our Godot RosMsg object
        Ref<RosMsg> wrapper;
        wrapper.instantiate();
        wrapper->init(msg);

        // 2. Safely hand it off to the Godot Main Thread
        callback_.call_deferred(wrapper);
    }
}