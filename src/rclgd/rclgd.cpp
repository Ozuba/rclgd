#include "rclgd.hpp"
#include <godot_cpp/variant/utility_functions.hpp>

rclgd *rclgd::singleton = nullptr;

rclgd *rclgd::get_singleton() { return singleton; }

rclgd::rclgd() {
    singleton = this;
}

rclgd::~rclgd() {
    shutdown();
    singleton = nullptr;
}

void rclgd::init(PackedStringArray args) {
    if (is_running_) return;

    // Convert Godot args to C-style args for rclcpp
    int argc = args.size();
    std::vector<char*> argv_vec;
    for (int i = 0; i < argc; ++i) {
        argv_vec.push_back(const_cast<char*>(args[i].utf8().get_data()));
    }

    rclcpp::init(argc, argv_vec.data());
    
    context_ = rclcpp::contexts::get_global_default_context();
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    
    is_running_ = true;
    spin_thread_ = std::thread([this]() {
        UtilityFunctions::print("ROS 2 Executor thread started.");
        executor_->spin();
        UtilityFunctions::print("ROS 2 Executor thread stopped.");
    });
}

void rclgd::shutdown() {
    if (!is_running_) return;
    
    is_running_ = false;
    rclcpp::shutdown(); // This causes executor_->spin() to return
    
    if (spin_thread_.joinable()) {
        spin_thread_.join();
    }
}

void rclgd::add_node(std::shared_ptr<rclcpp::Node> node) {
    std::lock_guard<std::mutex> lock(executor_mutex_);
    if (executor_) executor_->add_node(node);
}

void rclgd::remove_node(std::shared_ptr<rclcpp::Node> node) {
    std::lock_guard<std::mutex> lock(executor_mutex_);
    if (executor_) executor_->remove_node(node);
}

void rclgd::_bind_methods() {
    ClassDB::bind_method(D_METHOD("init", "args"), &rclgd::init);
    ClassDB::bind_method(D_METHOD("shutdown"), &rclgd::shutdown);
    ClassDB::bind_method(D_METHOD("ok"), &rclgd::ok);
}