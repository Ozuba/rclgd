#pragma once

#include <godot_cpp/classes/ref_counted.hpp>
#include <ros_babel_fish/babel_fish.hpp>
#include <rclcpp/rclcpp.hpp>
#include "ros_msg.hpp" // Your wrapper for BabelFish Message

using namespace godot;

class RosPublisher : public RefCounted {
    GDCLASS(RosPublisher, RefCounted);

private:
    std::shared_ptr<ros_babel_fish::BabelFishPublisher> pub_;

protected:
    static void _bind_methods();

public:
    RosPublisher() {}
    ~RosPublisher() {}

    // Called by RosNode::create_publisher
    void setup(const std::shared_ptr<rclcpp::Node> &node, const String &topic, const String &type);
    
    // The main function accessible from GDScript
    void publish(const Ref<RosMsg> &msg);
};

