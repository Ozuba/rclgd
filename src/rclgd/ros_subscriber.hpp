#pragma once

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/callable.hpp>
#include <ros_babel_fish/babel_fish.hpp>
#include <rclcpp/rclcpp.hpp>
#include "ros_msg.hpp"

using namespace godot;

class RosSubscriber : public RefCounted {
    GDCLASS(RosSubscriber, RefCounted);

private:
    std::shared_ptr<ros_babel_fish::BabelFishSubscription> sub_;
    Callable callback_;

protected:
    static void _bind_methods() {} // No methods to bind for GDScript usually

public:
    RosSubscriber() {}
    
    // Internal setup called by RosNode
    void setup(const std::shared_ptr<rclcpp::Node> &node, 
               const String &topic, 
               const String &type, 
               const Callable &p_callback);

    // The actual C++ callback that ROS triggers
    void _ros_callback(const ros_babel_fish::CompoundMessage::SharedPtr msg);
};

