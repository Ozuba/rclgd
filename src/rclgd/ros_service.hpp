#ifndef ROS_SERVICE_HPP
#define ROS_SERVICE_HPP

#include <godot_cpp/classes/ref_counted.hpp>
#include <ros_babel_fish/babel_fish.hpp>
#include "ros_msg.hpp"

using namespace godot;

class RosService : public RefCounted {
    GDCLASS(RosService, RefCounted);

private:
    ros_babel_fish::BabelFishService::SharedPtr service_;
    Callable callback_;

protected:
    static void _bind_methods();

public:
    RosService() = default;
    ~RosService() = default;

    // Factory setup
    void setup(std::shared_ptr<rclcpp::Node> p_node, const String &p_srv_name, const String &p_srv_type, const Callable &p_callback);
};

#endif