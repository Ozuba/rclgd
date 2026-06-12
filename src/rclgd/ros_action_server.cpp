#include "ros_action_server.hpp"
#include "rclgd.hpp"

#include <rclcpp_action/types.hpp>

using ros_babel_fish::BabelFishActionServerGoalHandle;
using ros_babel_fish::CompoundMessage;

/* ----------------------------- RosServerGoalHandle ----------------------------- */

void RosServerGoalHandle::_bind_methods()
{
    ClassDB::bind_method(D_METHOD("get_goal"), &RosServerGoalHandle::get_goal);
    ClassDB::bind_method(D_METHOD("get_goal_id"), &RosServerGoalHandle::get_goal_id);
    ClassDB::bind_method(D_METHOD("is_cancel_requested"), &RosServerGoalHandle::is_cancel_requested);
    ClassDB::bind_method(D_METHOD("is_active"), &RosServerGoalHandle::is_active);
    ClassDB::bind_method(D_METHOD("create_feedback"), &RosServerGoalHandle::create_feedback);
    ClassDB::bind_method(D_METHOD("create_result"), &RosServerGoalHandle::create_result);
    ClassDB::bind_method(D_METHOD("publish_feedback", "feedback"), &RosServerGoalHandle::publish_feedback);
    ClassDB::bind_method(D_METHOD("succeed", "result"), &RosServerGoalHandle::succeed);
    ClassDB::bind_method(D_METHOD("abort", "result"), &RosServerGoalHandle::abort);
    ClassDB::bind_method(D_METHOD("canceled", "result"), &RosServerGoalHandle::canceled);
}

void RosServerGoalHandle::_set_native(const std::shared_ptr<BabelFishActionServerGoalHandle> &p_handle)
{
    handle_ = p_handle;
    goal_.instantiate();
    auto goal = std::const_pointer_cast<CompoundMessage>(p_handle->get_goal());
    if (goal)
        goal_->init_babel(goal);
}

String RosServerGoalHandle::get_goal_id() const
{
    if (!handle_)
        return String();
    return String(rclcpp_action::to_string(handle_->get_goal_id()).c_str());
}

bool RosServerGoalHandle::is_cancel_requested() const
{
    return handle_ ? handle_->is_canceling() : false;
}

bool RosServerGoalHandle::is_active() const
{
    return handle_ ? handle_->is_active() : false;
}

Ref<RosMsg> RosServerGoalHandle::create_feedback()
{
    RCLGD_FAIL_COND_V_MSG(!handle_, nullptr, "RosServerGoalHandle: Handle not initialized.");
    Ref<RosMsg> fb;
    fb.instantiate();
    fb->init_babel(std::make_shared<CompoundMessage>(handle_->create_feedback_message()));
    return fb;
}

Ref<RosMsg> RosServerGoalHandle::create_result()
{
    RCLGD_FAIL_COND_V_MSG(!handle_, nullptr, "RosServerGoalHandle: Handle not initialized.");
    Ref<RosMsg> res;
    res.instantiate();
    res->init_babel(std::make_shared<CompoundMessage>(handle_->create_result_message()));
    return res;
}

void RosServerGoalHandle::publish_feedback(const Ref<RosMsg> &p_feedback)
{
    RCLGD_FAIL_COND_MSG(!handle_, "RosServerGoalHandle: Handle not initialized.");
    RCLGD_FAIL_COND_MSG(p_feedback.is_null() || !p_feedback->get_babel(), "RosServerGoalHandle: Invalid feedback message.");
    try
    {
        handle_->publish_feedback(*p_feedback->get_babel());
    }
    catch (const std::exception &e)
    {
        RCLGD_FAIL_MSG(vformat("RosServerGoalHandle: publish_feedback failed: %s", e.what()));
    }
}

void RosServerGoalHandle::succeed(const Ref<RosMsg> &p_result)
{
    RCLGD_FAIL_COND_MSG(!handle_, "RosServerGoalHandle: Handle not initialized.");
    RCLGD_FAIL_COND_MSG(p_result.is_null() || !p_result->get_babel(), "RosServerGoalHandle: Invalid result message.");
    try
    {
        handle_->succeed(*p_result->get_babel());
    }
    catch (const std::exception &e)
    {
        RCLGD_FAIL_MSG(vformat("RosServerGoalHandle: succeed failed (goal already terminal?): %s", e.what()));
    }
}

void RosServerGoalHandle::abort(const Ref<RosMsg> &p_result)
{
    RCLGD_FAIL_COND_MSG(!handle_, "RosServerGoalHandle: Handle not initialized.");
    RCLGD_FAIL_COND_MSG(p_result.is_null() || !p_result->get_babel(), "RosServerGoalHandle: Invalid result message.");
    try
    {
        handle_->abort(*p_result->get_babel());
    }
    catch (const std::exception &e)
    {
        RCLGD_FAIL_MSG(vformat("RosServerGoalHandle: abort failed (goal already terminal?): %s", e.what()));
    }
}

void RosServerGoalHandle::canceled(const Ref<RosMsg> &p_result)
{
    RCLGD_FAIL_COND_MSG(!handle_, "RosServerGoalHandle: Handle not initialized.");
    RCLGD_FAIL_COND_MSG(p_result.is_null() || !p_result->get_babel(), "RosServerGoalHandle: Invalid result message.");
    try
    {
        handle_->canceled(*p_result->get_babel());
    }
    catch (const std::exception &e)
    {
        RCLGD_FAIL_MSG(vformat("RosServerGoalHandle: canceled failed (goal already terminal?): %s", e.what()));
    }
}

/* ------------------------------- RosActionServer ------------------------------- */

void RosActionServer::setup(std::shared_ptr<rclcpp::Node> p_node, const String &p_action_name,
                            const String &p_action_type, const Callable &p_execute_callback)
{
    auto rclgd_ptr = rclgd::get_singleton();
    ERR_FAIL_NULL_MSG(rclgd_ptr, "RCLGD Singleton is null. Is the extension initialized?");

    // Captured by value so the executor-side callbacks never touch `this`
    // (same lifetime pattern as RosSubscriber).
    Callable callback = p_execute_callback;

    try
    {
        server_ = rclgd_ptr->get_fish().create_action_server(
            *p_node,
            p_action_name.utf8().get_data(),
            p_action_type.utf8().get_data(),
            // handle_goal: needs a synchronous answer on the executor thread,
            // so goals are auto-accepted instead of round-tripping to GDScript.
            [](const rclcpp_action::GoalUUID &, std::shared_ptr<const CompoundMessage>)
            {
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            // handle_cancel: auto-accept; the execute callback observes it
            // through RosServerGoalHandle.is_cancel_requested().
            [](const std::shared_ptr<BabelFishActionServerGoalHandle>)
            {
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            // handle_accepted: wrap the goal and hand it to GDScript on the
            // main thread.
            [callback](std::shared_ptr<BabelFishActionServerGoalHandle> handle)
            {
                if (!callback.is_valid())
                    return;
                Ref<RosServerGoalHandle> gh;
                gh.instantiate();
                gh->_set_native(handle);
                callback.call_deferred(gh);
            });
    }
    catch (const std::exception &e)
    {
        RCLGD_FAIL_MSG(vformat("RCLGD: Failed to create action server '%s' (%s): %s",
                               p_action_name, p_action_type, e.what()));
    }
}
