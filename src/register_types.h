#ifndef GDEXAMPLE_REGISTER_TYPES_H
#define GDEXAMPLE_REGISTER_TYPES_H

#include <godot_cpp/core/class_db.hpp>
//ROS2 for context init
#include <rclcpp/rclcpp.hpp>
//Modules
// RCLGD
#include "rclgd/rclgd.hpp"
#include "rclgd/ros_msg.hpp"
#include "rclgd/ros_node.hpp"
#include "rclgd/ros_publisher.hpp"
#include "rclgd/ros_subscriber.hpp"
#include "rclgd/ros_client.hpp"
#include "rclgd/ros_service.hpp"

//TF2 Helpers
#include "tf2gd/ros_node_3d.hpp"

using namespace godot;

void rclgd_init(ModuleInitializationLevel p_level);
void rclgd_deinit(ModuleInitializationLevel p_level);

#endif // GDEXAMPLE_REGISTER_TYPES_H