#ifndef ROS_ACTION_SERVER_HPP
#define ROS_ACTION_SERVER_HPP

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/callable.hpp>
#include <ros_babel_fish/babel_fish.hpp>
#include "ros_msg.hpp"

using namespace godot;

/*
 * Server-side handle for an accepted goal. Passed to the execute callback of
 * RosActionServer on the main thread. The handle must be driven to exactly
 * one terminal state: succeed(), abort() or canceled().
 */
class RosServerGoalHandle : public RefCounted
{
    GDCLASS(RosServerGoalHandle, RefCounted);

private:
    std::shared_ptr<ros_babel_fish::BabelFishActionServerGoalHandle> handle_;
    Ref<RosMsg> goal_;

protected:
    static void _bind_methods();

public:
    // Internal: called by RosActionServer
    void _set_native(const std::shared_ptr<ros_babel_fish::BabelFishActionServerGoalHandle> &p_handle);

    Ref<RosMsg> get_goal() const { return goal_; }
    String get_goal_id() const;

    // True once a client asked for this goal to be canceled. Poll it inside
    // long-running execute callbacks and respond with canceled().
    bool is_cancel_requested() const;
    bool is_active() const;

    Ref<RosMsg> create_feedback();
    Ref<RosMsg> create_result();

    void publish_feedback(const Ref<RosMsg> &p_feedback);

    // Terminal states
    void succeed(const Ref<RosMsg> &p_result);
    void abort(const Ref<RosMsg> &p_result);
    void canceled(const Ref<RosMsg> &p_result);
};

// Action server object, created through RosNode.create_action_server
class RosActionServer : public RefCounted
{
    GDCLASS(RosActionServer, RefCounted);

private:
    ros_babel_fish::BabelFishActionServer::SharedPtr server_;

protected:
    static void _bind_methods() {} // Configured entirely through setup

public:
    RosActionServer() = default;

    // Internal setup called by RosNode. Goals are auto-accepted and handed to
    // p_execute_callback on the main thread as a RosServerGoalHandle. Cancel
    // requests are auto-accepted too; the execute callback should poll
    // is_cancel_requested() and finish with canceled().
    void setup(std::shared_ptr<rclcpp::Node> p_node, const String &p_action_name,
               const String &p_action_type, const Callable &p_execute_callback);
};

#endif // ROS_ACTION_SERVER_HPP
