#pragma once

#ifndef RCLGD_HPP
#define RCLGD_HPP

#include <godot_cpp/classes/object.hpp>
#include <godot_cpp/core/class_db.hpp>

#include <ros_babel_fish/babel_fish.hpp>

#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <mutex>

using namespace godot;
using namespace ros_babel_fish;

class rclgd : public Object {
    GDCLASS(rclgd, Object);

private:
    static rclgd *singleton;
    
    std::shared_ptr<rclcpp::Context> context_;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    std::thread spin_thread_;
    std::atomic<bool> is_running_{false};
    std::mutex executor_mutex_;

    //Fish type support
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

    //Type support accesor
    BabelFish& get_fish() { return fish_; }
};

#endif