#include "rclgd.hpp"
#include "utils/ros_tf_utils.hpp"
#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/main_loop.hpp>
#include <godot_cpp/classes/scene_tree.hpp>
#include <godot_cpp/classes/window.hpp>
#include <godot_cpp/variant/callable.hpp>
#include <godot_cpp/variant/callable_method_pointer.hpp>

rclgd *rclgd::singleton = nullptr;
std::atomic<bool> rclgd::sigint_requested_{false};

rclgd *rclgd::get_singleton() { return singleton; }

void rclgd::handle_sigint(int sig) {
    // Signal handlers may only do async-signal-safe work: just raise a flag.
    // The quit is performed on the main thread in _on_physics_tick.
    sigint_requested_.store(true);
}

rclgd::rclgd()
{
    singleton = this;
}

rclgd::~rclgd()
{
    // Ensure the executor thread is joined and rclcpp is torn down even if
    // the user never called shutdown() explicitly.
    shutdown();
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
    // Store them permanently (rclcpp keeps pointers into these copies)
    for (int i = 0; i < args.size(); ++i)
        args_.push_back(std::string(args[i].utf8().get_data()));
    for (const auto &s : args_)
        argv_.push_back(const_cast<char *>(s.c_str()));

    //Register SIGINT HANDLER
    std::signal(SIGINT, rclgd::handle_sigint);

    rclcpp::InitOptions options;
    options.shutdown_on_signal = false; // Disable rclcpp's own SIGINT handlers
    rclcpp::init(static_cast<int>(argv_.size()), argv_.data(), options);

    // Hold context
    context_ = rclcpp::contexts::get_global_default_context();

    // Singleton Node
    rclcpp::NodeOptions node_options;
    node_options.allow_undeclared_parameters(true);
    node_options.automatically_declare_parameters_from_overrides(true);
    rclgd_node_ = std::make_shared<rclcpp::Node>("rclgd", node_options); // Node setup

    // Retrieve the values (Member variables)
    use_separate_thread_ = rclgd_node_->get_parameter_or<bool>("use_separate_thread", true);
    use_sim_time_ = rclgd_node_->get_parameter_or<bool>("use_sim_time", false);

    // Management of the executor context
    if (use_separate_thread_)
    { // Multithreaded Async Separate from GODOT Main thread for performance and throughput
        executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        spin_thread_.instantiate();
        spin_thread_->start(callable_mp(this, &rclgd::spin));
        UtilityFunctions::print("ROS 2 Executor running in separate thread");
    }
    else
    {
        // Synchronized with physics. One thread for deterministic usage
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        UtilityFunctions::print("ROS 2 Executor running in physics synchronous mode.");
    }

    // Add  Node to executor now its been created
    add_node(rclgd_node_); // Add node to executor

    // Instantiate SimTime Publisher
    auto sim_time_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();            // QOS for topic
    sim_time_pub_ = rclgd_node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", sim_time_qos); // Setup publisher

    // Get Godot Execution Loop
    SceneTree *tree = Object::cast_to<SceneTree>(Engine::get_singleton()->get_main_loop());

    // Attach RCLGD runtime to physics tick wether only publishes
    if (tree)
        tree->connect("physics_frame", callable_mp(this, &rclgd::_on_physics_tick));

    // Custom Performance monitors
    Performance::get_singleton()->add_custom_monitor(
        "ROS/Spin Time", 
        callable_mp(this, &rclgd::get_spin_time),
        {}, // Default arguments for the callable
        Performance::MONITOR_TYPE_TIME // This is the magic flag
    );

    // Set initialization okay
    is_running_ = true;

    // Emit signal for other systems to know ros2 context is ready
    emit_signal("ros_ready");
}

void rclgd::shutdown()
{
    if (!is_running_)
        return;

    if (executor_)
    {
        executor_->cancel();
    }

    if (spin_thread_.is_valid() && spin_thread_->is_started())
    {
        // Equivalent to std::thread::join()
        spin_thread_->wait_to_finish();

        // Clear the reference so the object is freed
        spin_thread_.unref();
        UtilityFunctions::print("ROS 2 Executor thread stopped cleanly.");
    }

    executor_.reset();
    sim_time_pub_.reset();
    rclgd_node_.reset();
    context_.reset();

    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }
    is_running_ = false;
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

void rclgd::spin()
{
    executor_->spin();
}

void rclgd::_on_physics_tick()
{
    // Handle Ctrl+C requested from the signal handler (main thread, safe to quit here)
    if (sigint_requested_.exchange(false))
    {
        MainLoop *main_loop = Engine::get_singleton()->get_main_loop();
        if (auto *tree = Object::cast_to<SceneTree>(main_loop))
        {
            tree->quit();
            return;
        }
    }

    if (use_sim_time_)
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

    // Make executor Spin some and consume available ros queues if sync mode
    if (!use_separate_thread_)
    {
        uint64_t start_usec = Time::get_singleton()->get_ticks_usec();
        {
            std::lock_guard<std::mutex> lock(executor_mutex_);

            executor_->spin_some();
        }
        uint64_t end_usec = Time::get_singleton()->get_ticks_usec();
        //Log for performance monitoring
        last_spin_time_us_ = end_usec - start_usec;
    }
}

Vector3 rclgd::godot_to_ros_vector(const Vector3 &p_v) const
{
    return RclgdUtils::godot_to_ros_vector(p_v);
}

Vector3 rclgd::ros_to_godot_vector(const Vector3 &p_v) const
{
    return RclgdUtils::ros_to_godot_vector(p_v);
}

Quaternion rclgd::godot_to_ros_quat(const Quaternion &p_q) const
{
    return RclgdUtils::godot_to_ros_quat(p_q);
}

Quaternion rclgd::ros_to_godot_quat(const Quaternion &p_q) const
{
    return RclgdUtils::ros_to_godot_quat(p_q);
}

void rclgd::_bind_methods()
{
    // Signals
    ADD_SIGNAL(MethodInfo("ros_ready"));

    // Interface
    ClassDB::bind_method(D_METHOD("init", "args"), &rclgd::init, DEFVAL(PackedStringArray()));
    ClassDB::bind_method(D_METHOD("shutdown"), &rclgd::shutdown);
    ClassDB::bind_method(D_METHOD("ok"), &rclgd::ok);

    // Coordinate convention helpers (same mapping used by TF broadcaster/listener)
    ClassDB::bind_method(D_METHOD("godot_to_ros_vector", "v"), &rclgd::godot_to_ros_vector);
    ClassDB::bind_method(D_METHOD("ros_to_godot_vector", "v"), &rclgd::ros_to_godot_vector);
    ClassDB::bind_method(D_METHOD("godot_to_ros_quat", "q"), &rclgd::godot_to_ros_quat);
    ClassDB::bind_method(D_METHOD("ros_to_godot_quat", "q"), &rclgd::ros_to_godot_quat);
}
