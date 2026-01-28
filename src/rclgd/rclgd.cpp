#include "rclgd.hpp"
#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/main_loop.hpp>
#include <godot_cpp/classes/scene_tree.hpp>
#include <godot_cpp/classes/window.hpp>
#include <godot_cpp/variant/callable.hpp>
#include <godot_cpp/variant/callable_method_pointer.hpp>

rclgd *rclgd::singleton = nullptr;

rclgd *rclgd::get_singleton() { return singleton; }

rclgd::rclgd()
{
    singleton = this;
}

rclgd::~rclgd()
{
    singleton = nullptr;
}

void rclgd::init(PackedStringArray args)
{
    if (is_running_)
        return;

    // Ensure arrays blank
    args_.clear();
    argv_.clear();

    // Convert Godot args to C-style args for rclcpp
    // Store them permanently
    for (int i = 0; i < args.size(); ++i)
        // Esto crea una COPIA persistente del texto
        args_.push_back(std::string(args[i].utf8().get_data()));
    for (const auto &s : args_)
        argv_.push_back(const_cast<char *>(s.c_str()));

    // ROS 2 init (usamos argv.size() - 1 para no contar el null en el argc)
    rclcpp::init(static_cast<int>(argv_.size()), argv_.data());

    context_ = rclcpp::contexts::get_global_default_context();
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    spin_thread_ = std::thread([this]()
                               {
        UtilityFunctions::print("ROS 2 Executor thread started.");
        executor_->spin();
        UtilityFunctions::print("ROS 2 Executor thread stopped."); });

    // Simulation time management
    rclgd_node_ = std::make_shared<rclcpp::Node>("rclgd"); // Node setup

    // Global parameter frame
    rclgd_node_->declare_parameter("global_frame", "map");

    // Add  Node to executor
    add_node(rclgd_node_); // Add node to executor

    // Instantiate SimTime Publisher
    auto sim_time_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();            // QOS for topic
    sim_time_pub_ = rclgd_node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", sim_time_qos); // Setup publisher

    // Connect to sim time if sim_time parameter is on
    SceneTree *tree = Object::cast_to<SceneTree>(Engine::get_singleton()->get_main_loop());
    if (tree && rclgd_node_->get_parameter("use_sim_time").as_bool())
    {
        // 2. Connect using a direct method pointer (Invisible to GDScript)
        tree->connect("physics_frame", callable_mp(this, &rclgd::_on_physics_tick));
    }

    // Set initialization okay
    is_running_ = true;
}

void rclgd::shutdown()
{
    if (!is_running_)
        return;

    if (executor_)
    {
        executor_->cancel();
    }

    if (spin_thread_.joinable())
    {
        spin_thread_.join();
    }

    executor_.reset();

    if (rclcpp::ok())
    {
        rclcpp::shutdown();
        is_running_ = false;
    }
}

void rclgd::add_node(std::shared_ptr<rclcpp::Node> node)
{
    std::lock_guard<std::mutex> lock(executor_mutex_);
    if (executor_)
        executor_->add_node(node);
}

void rclgd::remove_node(std::shared_ptr<rclcpp::Node> node)
{
    std::lock_guard<std::mutex> lock(executor_mutex_);
    if (executor_)
        executor_->remove_node(node);
}

void rclgd::_on_physics_tick()
{
    // Increment internal time
    int ticks_per_sec = Engine::get_singleton()->get_physics_ticks_per_second();
    double delta = (1.0 / static_cast<double>(ticks_per_sec)) * Engine::get_singleton()->get_time_scale();
    sim_time_ += delta;
    // Publish sim time
    auto msg = rosgraph_msgs::msg::Clock();
    msg.clock.sec = static_cast<int32_t>(sim_time_);
    msg.clock.nanosec = static_cast<uint32_t>((sim_time_ - msg.clock.sec) * 1e9);
    sim_time_pub_->publish(msg);
}

void rclgd::_bind_methods()
{
    // Interface
    ClassDB::bind_method(D_METHOD("init", "args"), &rclgd::init, DEFVAL(PackedStringArray()));
    ClassDB::bind_method(D_METHOD("shutdown"), &rclgd::shutdown);
    ClassDB::bind_method(D_METHOD("ok"), &rclgd::ok);
}
