#include "ros_logger.hpp"

#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/script_backtrace.hpp>
#include <godot_cpp/core/class_db.hpp>

#include <rcutils/logging.h>

// rcutils_log crashes on severities it doesn't know, so validate first.
static bool is_message_severity(int p_severity)
{
    switch (p_severity)
    {
    case RosLogger::SEVERITY_DEBUG:
    case RosLogger::SEVERITY_INFO:
    case RosLogger::SEVERITY_WARN:
    case RosLogger::SEVERITY_ERROR:
    case RosLogger::SEVERITY_FATAL:
        return true;
    default:
        return false;
    }
}

void RosLogger::_bind_methods()
{
    ClassDB::bind_method(D_METHOD("get_name"), &RosLogger::get_name);
    ClassDB::bind_method(D_METHOD("get_child", "name"), &RosLogger::get_child);

    ClassDB::bind_method(D_METHOD("set_level", "level"), &RosLogger::set_level);
    ClassDB::bind_method(D_METHOD("get_effective_level"), &RosLogger::get_effective_level);
    ClassDB::bind_method(D_METHOD("is_enabled_for", "severity"), &RosLogger::is_enabled_for);

    ClassDB::bind_method(D_METHOD("log", "message", "severity", "kwargs"), &RosLogger::log, DEFVAL(Dictionary()));
    ClassDB::bind_method(D_METHOD("debug", "message", "kwargs"), &RosLogger::debug, DEFVAL(Dictionary()));
    ClassDB::bind_method(D_METHOD("info", "message", "kwargs"), &RosLogger::info, DEFVAL(Dictionary()));
    ClassDB::bind_method(D_METHOD("warning", "message", "kwargs"), &RosLogger::warning, DEFVAL(Dictionary()));
    ClassDB::bind_method(D_METHOD("error", "message", "kwargs"), &RosLogger::error, DEFVAL(Dictionary()));
    ClassDB::bind_method(D_METHOD("fatal", "message", "kwargs"), &RosLogger::fatal, DEFVAL(Dictionary()));

    ADD_PROPERTY(PropertyInfo(Variant::STRING, "name"), "", "get_name");

    // Bound by hand so GDScript sees rclpy's names (RosLogger.DEBUG, ...)
    // without defining bare DEBUG/ERROR identifiers on the C++ side.
    const StringName cls = get_class_static();
    ClassDB::bind_integer_constant(cls, "LoggingSeverity", "UNSET", SEVERITY_UNSET);
    ClassDB::bind_integer_constant(cls, "LoggingSeverity", "DEBUG", SEVERITY_DEBUG);
    ClassDB::bind_integer_constant(cls, "LoggingSeverity", "INFO", SEVERITY_INFO);
    ClassDB::bind_integer_constant(cls, "LoggingSeverity", "WARN", SEVERITY_WARN);
    ClassDB::bind_integer_constant(cls, "LoggingSeverity", "ERROR", SEVERITY_ERROR);
    ClassDB::bind_integer_constant(cls, "LoggingSeverity", "FATAL", SEVERITY_FATAL);
}

Ref<RosLogger> RosLogger::wrap(const rclcpp::Logger &p_logger)
{
    Ref<RosLogger> logger;
    logger.instantiate();
    logger->logger_.emplace(p_logger);
    return logger;
}

String RosLogger::get_name() const
{
    if (!logger_ || !logger_->get_name())
        return String();
    return String::utf8(logger_->get_name());
}

Ref<RosLogger> RosLogger::get_child(const String &p_name) const
{
    RCLGD_FAIL_COND_V_MSG(!logger_, Ref<RosLogger>(), "RosLogger is not initialized; get one from RosNode.get_logger() or rclgd.get_logger().");
    RCLGD_FAIL_COND_V_MSG(p_name.is_empty(), Ref<RosLogger>(), "RCLGD: Child logger name must not be empty.");
    // rclcpp::Logger::get_child also registers the child as a /rosout
    // sublogger when the parent is a node logger.
    // get_child is non-const in rclcpp; copying the Logger is cheap.
    rclcpp::Logger parent = *logger_;
    return wrap(parent.get_child(p_name.utf8().get_data()));
}

void RosLogger::set_level(LoggingSeverity p_level)
{
    RCLGD_FAIL_COND_MSG(!logger_, "RosLogger is not initialized; get one from RosNode.get_logger() or rclgd.get_logger().");
    RCLGD_FAIL_COND_MSG(p_level != SEVERITY_UNSET && !is_message_severity(p_level),
                        vformat("RCLGD: Invalid logging level %d (expected a RosLogger.LoggingSeverity value).", p_level));
    if (rcutils_logging_set_logger_level(logger_->get_name(), p_level) != RCUTILS_RET_OK)
    {
        String reason = rcutils_get_error_string().str;
        rcutils_reset_error();
        RCLGD_FAIL_MSG(vformat("RCLGD: Failed to set level of logger '%s': %s", get_name(), reason));
    }
}

RosLogger::LoggingSeverity RosLogger::get_effective_level() const
{
    RCLGD_FAIL_COND_V_MSG(!logger_, SEVERITY_UNSET, "RosLogger is not initialized; get one from RosNode.get_logger() or rclgd.get_logger().");
    int level = rcutils_logging_get_logger_effective_level(logger_->get_name());
    return level < 0 ? SEVERITY_UNSET : static_cast<LoggingSeverity>(level);
}

bool RosLogger::is_enabled_for(LoggingSeverity p_severity) const
{
    RCLGD_FAIL_COND_V_MSG(!logger_, false, "RosLogger is not initialized; get one from RosNode.get_logger() or rclgd.get_logger().");
    return rcutils_logging_logger_is_enabled_for(logger_->get_name(), p_severity);
}

bool RosLogger::log(const String &p_message, LoggingSeverity p_severity, const Dictionary &p_kwargs)
{
    RCLGD_FAIL_COND_V_MSG(!logger_, false, "RosLogger is not initialized; get one from RosNode.get_logger() or rclgd.get_logger().");
    RCLGD_FAIL_COND_V_MSG(!is_message_severity(p_severity), false,
                          vformat("RCLGD: Invalid logging severity %d (expected DEBUG, INFO, WARN, ERROR or FATAL).", p_severity));

    // Parse the rclpy-style filter keyword arguments.
    bool once = false;
    bool skip_first = false;
    double throttle_duration_sec = 0.0;
    Array keys = p_kwargs.keys();
    for (int i = 0; i < keys.size(); i++)
    {
        const String key = keys[i];
        const Variant &value = p_kwargs[keys[i]];
        const Variant::Type type = value.get_type();
        if (key == "once" || key == "skip_first")
        {
            RCLGD_FAIL_COND_V_MSG(type != Variant::BOOL, false, vformat("RCLGD: Logging keyword argument '%s' must be a bool.", key));
            (key == "once" ? once : skip_first) = value;
        }
        else if (key == "throttle_duration_sec")
        {
            RCLGD_FAIL_COND_V_MSG(type != Variant::FLOAT && type != Variant::INT, false, "RCLGD: Logging keyword argument 'throttle_duration_sec' must be a number.");
            throttle_duration_sec = value;
        }
        else
            RCLGD_FAIL_V_MSG(false, vformat("RCLGD: Unsupported logging keyword argument '%s' (expected once, skip_first or throttle_duration_sec).", key));
    }
    const bool throttle = throttle_duration_sec > 0.0;
    RCLGD_FAIL_COND_V_MSG(once && (skip_first || throttle), false, "RCLGD: The 'once' logging filter cannot be combined with other filters.");

    const char *name = logger_->get_name();
    if (!rcutils_logging_logger_is_enabled_for(name, p_severity))
        return false;

    // Locate the GDScript caller, used both as the log location and as the
    // filter key. Script call stacks are unavailable in release exports
    // unless debug/settings/gdscript/always_track_call_stacks is enabled;
    // filters then fall back to keying on the message text.
    std::string function, file;
    size_t line = 0;
    TypedArray<ScriptBacktrace> backtraces = Engine::get_singleton()->capture_script_backtraces(false);
    for (int i = 0; i < backtraces.size(); i++)
    {
        Ref<ScriptBacktrace> bt = backtraces[i];
        if (bt.is_valid() && bt->get_frame_count() > 0)
        {
            function = bt->get_frame_function(0).utf8().get_data();
            file = bt->get_frame_file(0).utf8().get_data();
            line = bt->get_frame_line(0);
            break;
        }
    }

    if (once || skip_first || throttle)
    {
        std::string call_site = file.empty()
                                    ? std::string("msg:") + p_message.utf8().get_data()
                                    : file + ":" + std::to_string(line) + ":" + function;

        std::lock_guard<std::mutex> lock(contexts_mutex_);
        if (file.empty())
        {
            if (!warned_no_call_site_)
            {
                warned_no_call_site_ = true;
                WARN_PRINT("RCLGD: Script call stacks are not tracked, so logging filters key on the message text. Enable debug/settings/gdscript/always_track_call_stacks for per-call-site filtering.");
            }
            if (contexts_.size() >= MAX_TEXT_KEYED_CONTEXTS)
                contexts_.clear();
        }
        CallSiteContext &ctx = contexts_[call_site];

        // Same order as rclpy: throttle, then skip_first, then once. A
        // suppressed message still updates the filter state, like rcutils.
        if (throttle)
        {
            auto now = std::chrono::steady_clock::now();
            if (ctx.throttle_last_logged &&
                now - *ctx.throttle_last_logged < std::chrono::duration<double>(throttle_duration_sec))
                return false;
            ctx.throttle_last_logged = now;
        }
        if (skip_first && !ctx.first_skipped)
        {
            ctx.first_skipped = true;
            return false;
        }
        if (once)
        {
            if (ctx.has_been_logged_once)
                return false;
            ctx.has_been_logged_once = true;
        }
    }

    rcutils_log_location_t location{function.c_str(), file.c_str(), line};
    rcutils_log(file.empty() ? nullptr : &location, p_severity, name, "%s", p_message.utf8().get_data());
    return true;
}
