#include "ros_timer.hpp"
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/callable.hpp>

using namespace godot;

void RosTimer::_bind_methods() {
    ClassDB::bind_method(D_METHOD("cancel"), &RosTimer::cancel);
    ClassDB::bind_method(D_METHOD("reset"), &RosTimer::reset);
    ClassDB::bind_method(D_METHOD("is_canceled"), &RosTimer::is_canceled);
    ClassDB::bind_method(D_METHOD("is_ready"), &RosTimer::is_ready);
    ClassDB::bind_method(D_METHOD("get_remaining_time"), &RosTimer::get_remaining_time);
}

void RosTimer::setup(const std::shared_ptr<rclcpp::Node> &node, double p_seconds, const Callable &p_callable) {
    if (!node) return;

    // Convert seconds to nanoseconds for ROS 2 duration
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(p_seconds)
    );

    // Create the timer. p_callable is captured by value for thread safety.
    _timer_ptr = node->create_wall_timer(duration, [p_callable]() {
        if (p_callable.is_valid()) {
            // Jump from ROS executor thread back to Godot main thread
            p_callable.call_deferred();
        }
    });
}

void RosTimer::cancel() {
    if (_timer_ptr) {
        _timer_ptr->cancel();
    }
}

void RosTimer::reset() {
    if (_timer_ptr) {
        _timer_ptr->reset();
    }
}

bool RosTimer::is_canceled() const {
    return _timer_ptr ? _timer_ptr->is_canceled() : true;
}

bool RosTimer::is_ready() const {
    return _timer_ptr ? _timer_ptr->is_ready() : false;
}

double RosTimer::get_remaining_time() const {
    if (!_timer_ptr) return 0.0;
    auto remaining = _timer_ptr->time_until_trigger();
    return std::chrono::duration<double>(remaining).count();
}