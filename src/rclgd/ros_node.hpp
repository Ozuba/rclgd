#pragma once

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros_publisher.hpp"
#include "ros_subscriber.hpp"

using namespace godot;

class RosNode : public RefCounted {
    GDCLASS(RosNode, RefCounted)

private:
    std::shared_ptr<rclcpp::Node> node_;

protected:
    static void _bind_methods();

public:
    RosNode() {}
    ~RosNode();

    // Manual initialization since we aren't in the SceneTree
    void init(const String &p_node_name);

    // Publisher and subscriber creation
    Ref<RosPublisher> create_publisher(const String &topic, const String &type);
    Ref<RosSubscriber> create_subscriber(const String &topic, const String &type, const Callable &callback);

    //Time related
    Ref<RosMsg> now();
};