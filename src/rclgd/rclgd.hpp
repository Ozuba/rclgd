#pragma once

#ifndef RCLGD_HPP
#define RCLGD_HPP

#include <godot_cpp/classes/object.hpp>
#include <godot_cpp/core/class_db.hpp>

#include <ros_babel_fish/babel_fish.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <thread>
#include <mutex>

using namespace godot;
using namespace ros_babel_fish;

class rclgd : public Object
{
    GDCLASS(rclgd, Object);

private:
    static rclgd *singleton;

    // Context
    // Init arguments
    std::vector<std::string> args_;
    std::vector<char *> argv_;

    // Context and Executor
    std::shared_ptr<rclcpp::Context> context_;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    std::thread spin_thread_;
    std::atomic<bool> is_running_{false};
    std::mutex executor_mutex_;

    // Simulation Node
    std::shared_ptr<rclcpp::Node> rclgd_node_;

    // Simulation Time
    double sim_time_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr sim_time_pub_;
    void _on_physics_tick();

    // Transform  Support
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

    // Fish type support
    BabelFish fish_;

protected:
    static void _bind_methods();

public:
    static rclgd *get_singleton();

    rclgd();
    ~rclgd();

    // Context & Executor Management
    void init(PackedStringArray args);
    void shutdown();

    // Node Registry (Called by RosNode C++ classes)
    void add_node(std::shared_ptr<rclcpp::Node> node);
    void remove_node(std::shared_ptr<rclcpp::Node> node);

    bool ok() const { return is_running_; }

    
    // RCLGD node accesor
    std::shared_ptr<rclcpp::Node> get_rclgd_node() const{ return rclgd_node_;}
    //Transform publisher accessor
    std::shared_ptr<tf2_ros::TransformBroadcaster> get_tf_broadcaster() { return tf_broadcaster;}
    //TF Buffer accessor
    std::shared_ptr<tf2_ros::Buffer> get_tf_buffer() { return tf_buffer; }

    // Type support accesor
    BabelFish &get_fish() { return fish_; }


};

#endif