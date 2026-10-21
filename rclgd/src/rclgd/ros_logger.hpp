#pragma once

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/dictionary.hpp>

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

#include "utils/rclgd_macros.hpp"

using namespace godot;

// Mirrors rclpy's RcutilsLogger: obtained from RosNode.get_logger() or
// rclgd.get_logger(name), never constructed directly from GDScript.
class RosLogger : public RefCounted
{
    GDCLASS(RosLogger, RefCounted)

public:
    // Values mirror RCUTILS_LOG_SEVERITY / rclpy.logging.LoggingSeverity.
    // Bound to GDScript as UNSET, DEBUG, ... (see _bind_methods).
    enum LoggingSeverity
    {
        SEVERITY_UNSET = 0,
        SEVERITY_DEBUG = 10,
        SEVERITY_INFO = 20,
        SEVERITY_WARN = 30,
        SEVERITY_ERROR = 40,
        SEVERITY_FATAL = 50,
    };

private:
    // rclcpp::Logger has no public default constructor. Holding the rclcpp
    // object (not just the name) keeps child loggers registered as /rosout
    // subloggers for as long as this RosLogger lives.
    std::optional<rclcpp::Logger> logger_;

    // Per-call-site state for the once / skip_first / throttle filters,
    // keyed by the GDScript caller like rclpy keys by caller frame.
    struct CallSiteContext
    {
        bool has_been_logged_once = false;
        bool first_skipped = false;
        std::optional<std::chrono::steady_clock::time_point> throttle_last_logged;
    };
    std::unordered_map<std::string, CallSiteContext> contexts_;
    std::mutex contexts_mutex_;
    bool warned_no_call_site_ = false;

    // Without script call stacks, filters key on the message text, so
    // messages with changing values would grow contexts_ forever.
    static constexpr size_t MAX_TEXT_KEYED_CONTEXTS = 1024;

protected:
    static void _bind_methods();

public:
    RosLogger() = default;
    ~RosLogger() = default;

    static Ref<RosLogger> wrap(const rclcpp::Logger &p_logger);

    String get_name() const;
    Ref<RosLogger> get_child(const String &p_name) const;

    void set_level(LoggingSeverity p_level);
    LoggingSeverity get_effective_level() const;
    bool is_enabled_for(LoggingSeverity p_severity) const;

    bool log(const String &p_message, LoggingSeverity p_severity, const Dictionary &p_kwargs = Dictionary());
    bool debug(const String &p_message, const Dictionary &p_kwargs = Dictionary()) { return log(p_message, SEVERITY_DEBUG, p_kwargs); }
    bool info(const String &p_message, const Dictionary &p_kwargs = Dictionary()) { return log(p_message, SEVERITY_INFO, p_kwargs); }
    bool warning(const String &p_message, const Dictionary &p_kwargs = Dictionary()) { return log(p_message, SEVERITY_WARN, p_kwargs); }
    bool error(const String &p_message, const Dictionary &p_kwargs = Dictionary()) { return log(p_message, SEVERITY_ERROR, p_kwargs); }
    bool fatal(const String &p_message, const Dictionary &p_kwargs = Dictionary()) { return log(p_message, SEVERITY_FATAL, p_kwargs); }
};

VARIANT_ENUM_CAST(RosLogger::LoggingSeverity);
