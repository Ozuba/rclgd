#include "ros_subscriber.hpp"
#include "rclgd.hpp"

void RosSubscriber::setup(const std::shared_ptr<rclcpp::Node> &node,
                          const String &topic,
                          const String &type,
                          const Callable &p_callback,
                          const Ref<RosQoS> &qos)
{
    std::string std_topic = topic.utf8().get_data();
    std::string std_type = type.utf8().get_data();
    // No RosQoS given -> the ROS default topic profile (keep-last 10)
    rclcpp::QoS final_qos = qos.is_valid() ? qos->get_qos() : rclcpp::QoS(10);
    auto &fish = rclgd::get_singleton()->get_fish();

    // The Callable is captured by value so the executor-side callback never
    // touches `this`: if GDScript drops the last reference to this subscriber
    // while a callback is in flight, nothing dangles.
    Callable callback = p_callback;
    try
    {
        sub_ = fish.create_subscription(
            *node, std_topic, std_type, final_qos,
            [callback](const ros_babel_fish::CompoundMessage::SharedPtr msg)
            {
                if (!callback.is_valid())
                    return;

                // Wrap the raw BabelFish message into our Godot RosMsg object
                Ref<RosMsg> wrapper;
                wrapper.instantiate();
                wrapper->init_babel(msg);

                // Call immediately or defer depending on the thread we are in
                if (Thread::is_main_thread())
                    callback.call(wrapper);
                else
                    callback.call_deferred(wrapper);
            });
    }
    catch (const std::exception &e)
    {
        RCLGD_FAIL_MSG(vformat("RCLGD Subscription to '%s' (%s) failed: %s", topic, type, e.what()));
    }
}
