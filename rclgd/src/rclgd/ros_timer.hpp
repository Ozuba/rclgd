#ifndef ROS_TIMER_HPP
#define ROS_TIMER_HPP

#include <godot_cpp/classes/ref_counted.hpp>
#include <rclcpp/rclcpp.hpp>

namespace godot {

class RosTimer : public RefCounted {
    GDCLASS(RosTimer, RefCounted);

private:
    rclcpp::TimerBase::SharedPtr _timer_ptr;

protected:
    static void _bind_methods();

public:
    RosTimer() = default;
    ~RosTimer() = default;

    // Setup function to initialize the rclcpp timer
    void setup(const std::shared_ptr<rclcpp::Node> &node, double p_seconds, const Callable &p_callable);

    // Standard rclcpp lifecycle methods
    void cancel();
    void reset();
    bool is_canceled() const;
    bool is_ready() const;
    
    // Standard rclcpp state methods
    double get_remaining_time() const;
};

} // namespace godot

#endif