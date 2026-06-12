#pragma once

#ifndef RCLGD_HPP
#define RCLGD_HPP

#include <godot_cpp/classes/object.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/os.hpp>
#include <godot_cpp/classes/thread.hpp>
#include <godot_cpp/classes/performance.hpp>
#include <godot_cpp/classes/time.hpp>
#include <godot_cpp/variant/vector3.hpp>
#include <godot_cpp/variant/quaternion.hpp>

#include <ros_babel_fish/babel_fish.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <rclcpp/rclcpp.hpp>

#include <thread>
#include <mutex>
#include <csignal>

//So all other classes have access to macros
#include "utils/rclgd_macros.hpp"

using namespace godot;
using namespace ros_babel_fish;

class rclgd : public Object
{
    GDCLASS(rclgd, Object);

private:
    static rclgd *singleton;

    // Options
    bool use_separate_thread_ = false; // Run Ros spinning in separate thread
    bool use_sim_time_ = false;        // Use Simulation time

    // Context
    // Init arguments
    std::vector<std::string> args_;
    std::vector<char *> argv_;

    // Sigint handling for rapid closure in ros runtimes.
    // The handler only sets the flag (async-signal-safe); the actual quit
    // happens on the main thread in _on_physics_tick.
    static std::atomic<bool> sigint_requested_;
    static void handle_sigint(int sig);

    // Context and Executor
    std::shared_ptr<rclcpp::Context> context_;
    std::shared_ptr<rclcpp::Executor> executor_;
    // Separate thread mode
    std::atomic<bool> is_running_{false};
    Ref<Thread> spin_thread_;

    void spin();             // Spinner for thread
    void _on_physics_tick(); // Runtime callback

    std::mutex executor_mutex_;
    // Performance monitoring
    uint64_t last_spin_time_us_ = 0;

    // Simulation Node
    std::shared_ptr<rclcpp::Node> rclgd_node_;

    // Simulation Time
    double sim_time_ = 0;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr sim_time_pub_;

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

    // RCLGD node accessor
    std::shared_ptr<rclcpp::Node> get_rclgd_node() const { return rclgd_node_; }

    // For performance monitoring
    double get_spin_time() const { return static_cast<double>(last_spin_time_us_) / 1000000.0; }
    // Type support accessor
    BabelFish &get_fish() { return fish_; }

    // Coordinate convention helpers (Godot Y-up/-Z-forward <-> ROS Z-up/X-forward).
    // Exposed to GDScript so user code converting poses/twists by hand uses the
    // exact same mapping as the TF broadcaster/listener.
    Vector3 godot_to_ros_vector(const Vector3 &p_v) const;
    Vector3 ros_to_godot_vector(const Vector3 &p_v) const;
    Quaternion godot_to_ros_quat(const Quaternion &p_q) const;
    Quaternion ros_to_godot_quat(const Quaternion &p_q) const;
};

#endif