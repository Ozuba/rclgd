#ifndef ROS_ACTION_CLIENT_HPP
#define ROS_ACTION_CLIENT_HPP

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/classes/thread.hpp>
#include <ros_babel_fish/babel_fish.hpp>
#include "ros_msg.hpp"
#include "ros_qos.hpp"

#include <mutex>

using namespace godot;

/*
 * Client-side handle for a goal sent through RosActionClient.
 * Behaves like a promise: poll is_completed()/get_result() or await the
 * signals. All signals are emitted on the Godot main thread (the executor
 * thread dispatches through call_deferred), so GDScript never races.
 */
class RosGoalHandle : public RefCounted
{
    GDCLASS(RosGoalHandle, RefCounted);

public:
    // Matches rclcpp_action::ResultCode (which mirrors action_msgs/GoalStatus)
    enum Status {
        STATUS_UNKNOWN = 0,
        STATUS_SUCCEEDED = 4,
        STATUS_ABORTED = 5,
        STATUS_CANCELED = 6,
    };

private:
    using BabelGoalHandle = ros_babel_fish::BabelFishActionClient::GoalHandle;

    // The native handle is written from the executor thread (goal response)
    // and read from the main thread (cancel), hence the mutex.
    std::mutex native_mutex_;
    BabelGoalHandle::SharedPtr handle_;
    ros_babel_fish::BabelFishActionClient::SharedPtr client_;

    // Only mutated on the main thread (deferred entry points below)
    bool completed_ = false;
    bool accepted_ = false;
    int status_ = STATUS_UNKNOWN;
    Ref<RosMsg> result_;

protected:
    static void _bind_methods();

public:
    bool is_completed() const { return completed_; }
    bool is_accepted() const { return accepted_; }
    int get_status() const { return status_; }
    Ref<RosMsg> get_result() const { return result_; }

    // Ask the server to cancel this goal. The outcome still arrives through
    // the "completed" signal (with STATUS_CANCELED) once the server reacts.
    void cancel();

    // Internal: called by RosActionClient from the executor thread.
    void _set_native(const BabelGoalHandle::SharedPtr &p_handle,
                     const ros_babel_fish::BabelFishActionClient::SharedPtr &p_client);

    // Main-thread entry points, dispatched via call_deferred from the executor.
    void _on_goal_response(bool p_accepted);
    void _on_feedback(const Ref<RosMsg> &p_feedback);
    void _on_result(const Ref<RosMsg> &p_result, int p_status);
};

VARIANT_ENUM_CAST(RosGoalHandle::Status);

// Action client object, created through RosNode.create_action_client
class RosActionClient : public RefCounted
{
    GDCLASS(RosActionClient, RefCounted);

private:
    ros_babel_fish::BabelFishActionClient::SharedPtr client_;
    std::string action_type_;

    // Background Godot Thread backing wait_for_server_async.
    Ref<Thread> wait_thread_;
    void _run_wait_for_server(double p_timeout_sec); // thread body
    void _finish_wait_for_server(bool p_available);  // main thread: join + emit

protected:
    static void _bind_methods();

public:
    RosActionClient() = default;
    ~RosActionClient();

    // Internal setup called by RosNode
    void setup(std::shared_ptr<rclcpp::Node> p_node, const String &p_action_name, const String &p_action_type, const Ref<RosQoS> &p_qos = Ref<RosQoS>());

    // Blocking: stalls the calling thread for up to p_timeout_sec.
    bool wait_for_server(double p_timeout_sec);
    // Non-blocking: emits "server_available(bool)" on the main thread once the
    // action server appears or the timeout elapses. Await it directly.
    void wait_for_server_async(double p_timeout_sec);
    bool is_server_ready() const;
    Ref<RosMsg> create_goal();
    Ref<RosGoalHandle> send_goal(const Ref<RosMsg> &p_goal);
};

#endif // ROS_ACTION_CLIENT_HPP
