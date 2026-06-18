#include "ros_action_client.hpp"
#include "rclgd.hpp"

#include <godot_cpp/variant/callable_method_pointer.hpp>

using BabelGoalHandle = ros_babel_fish::BabelFishActionClient::GoalHandle;

/* -------------------------------- RosGoalHandle -------------------------------- */

void RosGoalHandle::_bind_methods()
{
    ClassDB::bind_method(D_METHOD("is_completed"), &RosGoalHandle::is_completed);
    ClassDB::bind_method(D_METHOD("is_accepted"), &RosGoalHandle::is_accepted);
    ClassDB::bind_method(D_METHOD("get_status"), &RosGoalHandle::get_status);
    ClassDB::bind_method(D_METHOD("get_result"), &RosGoalHandle::get_result);
    ClassDB::bind_method(D_METHOD("cancel"), &RosGoalHandle::cancel);

    // Bound so the executor thread can dispatch through call_deferred
    ClassDB::bind_method(D_METHOD("_on_goal_response", "accepted"), &RosGoalHandle::_on_goal_response);
    ClassDB::bind_method(D_METHOD("_on_feedback", "feedback"), &RosGoalHandle::_on_feedback);
    ClassDB::bind_method(D_METHOD("_on_result", "result", "status"), &RosGoalHandle::_on_result);

    ADD_SIGNAL(MethodInfo("accepted"));
    ADD_SIGNAL(MethodInfo("rejected"));
    ADD_SIGNAL(MethodInfo("feedback", PropertyInfo(Variant::OBJECT, "msg", PROPERTY_HINT_RESOURCE_TYPE, "RosMsg")));
    ADD_SIGNAL(MethodInfo("completed", PropertyInfo(Variant::OBJECT, "result", PROPERTY_HINT_RESOURCE_TYPE, "RosMsg"),
                          PropertyInfo(Variant::INT, "status")));

    BIND_ENUM_CONSTANT(STATUS_UNKNOWN);
    BIND_ENUM_CONSTANT(STATUS_SUCCEEDED);
    BIND_ENUM_CONSTANT(STATUS_ABORTED);
    BIND_ENUM_CONSTANT(STATUS_CANCELED);
}

void RosGoalHandle::_set_native(const BabelGoalHandle::SharedPtr &p_handle,
                                const ros_babel_fish::BabelFishActionClient::SharedPtr &p_client)
{
    std::lock_guard<std::mutex> lock(native_mutex_);
    handle_ = p_handle;
    client_ = p_client;
}

void RosGoalHandle::cancel()
{
    if (completed_)
        return;

    BabelGoalHandle::SharedPtr handle;
    ros_babel_fish::BabelFishActionClient::SharedPtr client;
    {
        std::lock_guard<std::mutex> lock(native_mutex_);
        handle = handle_;
        client = client_;
    }
    RCLGD_FAIL_COND_MSG(!handle || !client, "RosGoalHandle: Cannot cancel, the goal has not been accepted (yet).");

    try
    {
        client->async_cancel_goal(handle);
    }
    catch (const std::exception &e)
    {
        RCLGD_FAIL_MSG(vformat("RosGoalHandle: Cancel request failed: %s", e.what()));
    }
}

void RosGoalHandle::_on_goal_response(bool p_accepted)
{
    accepted_ = p_accepted;
    if (p_accepted)
    {
        emit_signal("accepted");
    }
    else
    {
        // A rejected goal will never produce a result: settle the promise here
        // so 'await completed' never hangs.
        emit_signal("rejected");
        completed_ = true;
        status_ = STATUS_UNKNOWN;
        emit_signal("completed", Ref<RosMsg>(), status_);
    }
}

void RosGoalHandle::_on_feedback(const Ref<RosMsg> &p_feedback)
{
    emit_signal("feedback", p_feedback);
}

void RosGoalHandle::_on_result(const Ref<RosMsg> &p_result, int p_status)
{
    result_ = p_result;
    status_ = p_status;
    completed_ = true;

    // Drop the native handle: it stores the callbacks that capture a Ref to
    // this object, so keeping it past the terminal state would create a
    // reference cycle and leak both objects.
    {
        std::lock_guard<std::mutex> lock(native_mutex_);
        handle_.reset();
        client_.reset();
    }

    emit_signal("completed", result_, status_);
}

/* -------------------------------- RosActionClient ------------------------------ */

void RosActionClient::_bind_methods()
{
    ClassDB::bind_method(D_METHOD("wait_for_server", "timeout_sec"), &RosActionClient::wait_for_server);
    ClassDB::bind_method(D_METHOD("wait_for_server_async", "timeout_sec"), &RosActionClient::wait_for_server_async);
    ClassDB::bind_method(D_METHOD("is_server_ready"), &RosActionClient::is_server_ready);
    ClassDB::bind_method(D_METHOD("create_goal"), &RosActionClient::create_goal);
    ClassDB::bind_method(D_METHOD("send_goal", "goal"), &RosActionClient::send_goal);

    // Emitted on the main thread by wait_for_server_async; await it directly.
    ADD_SIGNAL(MethodInfo("server_available", PropertyInfo(Variant::BOOL, "available")));
}

void RosActionClient::setup(std::shared_ptr<rclcpp::Node> p_node, const String &p_action_name, const String &p_action_type, const Ref<RosQoS> &p_qos)
{
    action_type_ = p_action_type.utf8().get_data();

    auto rclgd_ptr = rclgd::get_singleton();
    ERR_FAIL_NULL_MSG(rclgd_ptr, "RCLGD Singleton is null. Is the extension initialized?");

    // Start from rcl defaults and, if a profile was given, apply it to the
    // request/response/feedback channels. status_topic_qos is left at its
    // default (transient_local) so late-joining clients still get goal status.
    rcl_action_client_options_t options = rcl_action_client_get_default_options();
    if (p_qos.is_valid())
    {
        rmw_qos_profile_t profile = p_qos->get_qos().get_rmw_qos_profile();
        options.goal_service_qos = profile;
        options.result_service_qos = profile;
        options.cancel_service_qos = profile;
        options.feedback_topic_qos = profile;
    }

    try
    {
        client_ = rclgd_ptr->get_fish().create_action_client(
            *p_node,
            p_action_name.utf8().get_data(),
            action_type_,
            options);
    }
    catch (const std::exception &e)
    {
        RCLGD_FAIL_MSG(vformat("RCLGD: Failed to create action client '%s' (%s): %s",
                               p_action_name, p_action_type, e.what()));
    }
}

bool RosActionClient::wait_for_server(double p_timeout_sec)
{
    ERR_FAIL_COND_V_EDMSG(!client_, false, "Cannot wait_for_server: Action client is not initialized.");
    return client_->wait_for_action_server(std::chrono::duration<double>(p_timeout_sec));
}

void RosActionClient::wait_for_server_async(double p_timeout_sec)
{
    if (!client_)
    {
        call_deferred("emit_signal", "server_available", false);
        return;
    }
    if (wait_thread_.is_valid() && wait_thread_->is_started())
    {
        WARN_PRINT_ED("RosActionClient: a wait_for_server_async is already in progress.");
        return;
    }

    // Run the blocking wait on a Godot Thread; the result is marshalled back to
    // the main thread where the worker is joined and the signal emitted.
    wait_thread_.instantiate();
    wait_thread_->start(callable_mp(this, &RosActionClient::_run_wait_for_server).bind(p_timeout_sec));
}

void RosActionClient::_run_wait_for_server(double p_timeout_sec)
{
    bool ok = client_ && client_->wait_for_action_server(std::chrono::duration<double>(p_timeout_sec));
    callable_mp(this, &RosActionClient::_finish_wait_for_server).call_deferred(ok);
}

void RosActionClient::_finish_wait_for_server(bool p_available)
{
    if (wait_thread_.is_valid())
    {
        wait_thread_->wait_to_finish();
        wait_thread_.unref();
    }
    emit_signal("server_available", p_available);
}

RosActionClient::~RosActionClient()
{
    // Join an in-flight wait so the Godot Thread is cleaned up on destruction.
    if (wait_thread_.is_valid() && wait_thread_->is_started())
        wait_thread_->wait_to_finish();
}

bool RosActionClient::is_server_ready() const
{
    if (!client_)
        return false;
    return client_->action_server_is_ready();
}

Ref<RosMsg> RosActionClient::create_goal()
{
    ERR_FAIL_COND_V_EDMSG(action_type_.empty(), nullptr, "Cannot create_goal: Action type is unknown. Did setup() fail?");

    auto rclgd_ptr = rclgd::get_singleton();
    ERR_FAIL_NULL_V_EDMSG(rclgd_ptr, nullptr, "RCLGD Singleton vanished during goal creation.");

    try
    {
        auto goal_msg = rclgd_ptr->get_fish().create_action_goal_shared(action_type_);
        Ref<RosMsg> goal;
        goal.instantiate();
        goal->init_babel(goal_msg);
        return goal;
    }
    catch (const std::exception &e)
    {
        RCLGD_FAIL_V_MSG(Ref<RosMsg>(), vformat("BabelFish failed to create goal for type '%s': %s",
                                                String(action_type_.c_str()), e.what()));
    }
}

Ref<RosGoalHandle> RosActionClient::send_goal(const Ref<RosMsg> &p_goal)
{
    RCLGD_FAIL_COND_V_MSG(!client_, nullptr, "RosActionClient: Action client not initialized.");
    RCLGD_FAIL_COND_V_MSG(p_goal.is_null() || !p_goal->get_babel(), nullptr, "RosActionClient: Invalid goal message.");

    // Create the "Promise" object. The lambdas below capture it by value
    // (keeping it alive) and only touch it through call_deferred, so all
    // GDScript-visible state changes happen on the main thread.
    Ref<RosGoalHandle> gh;
    gh.instantiate();

    ros_babel_fish::BabelFishActionClient::SendGoalOptions options;
    auto client = client_;

    options.goal_response_callback = [gh, client](BabelGoalHandle::SharedPtr handle)
    {
        if (handle)
            gh->_set_native(handle, client); // Thread-safe, needed for cancel()
        gh->call_deferred("_on_goal_response", handle != nullptr);
    };

    options.feedback_callback = [gh](BabelGoalHandle::SharedPtr,
                                     const std::shared_ptr<const ros_babel_fish::CompoundMessage> feedback)
    {
        if (!feedback)
            return;
        Ref<RosMsg> fb;
        fb.instantiate();
        fb->init_babel(std::const_pointer_cast<ros_babel_fish::CompoundMessage>(feedback));
        gh->call_deferred("_on_feedback", fb);
    };

    options.result_callback = [gh](const BabelGoalHandle::WrappedResult &result)
    {
        Ref<RosMsg> res;
        res.instantiate();
        if (result.result)
            res->init_babel(result.result);
        gh->call_deferred("_on_result", res, (int)result.code);
    };

    try
    {
        client_->async_send_goal(*p_goal->get_babel(), options);
    }
    catch (const std::exception &e)
    {
        RCLGD_FAIL_V_MSG(Ref<RosGoalHandle>(), vformat("RosActionClient: Failed to send goal: %s", e.what()));
    }

    return gh;
}
