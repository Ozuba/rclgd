#ifndef RCLGD_MACROS_HPP
#define RCLGD_MACROS_HPP

#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/engine_debugger.hpp>
#include <godot_cpp/classes/project_settings.hpp>
#include <godot_cpp/classes/script_language.hpp>
#include <godot_cpp/core/error_macros.hpp>

/**
 * THE RCLGD FAIL MACRO (Clean Version)
 * 1. Prints clickable C++ error via native GDExtension macros.
 * 2. Optionally triggers script_debug to pause execution, gated behind the
 *    "rclgd/debug/break_on_error" project setting (default: true) so users
 *    can keep running through recoverable runtime errors.
 */
#define _RCLGD_FAIL_IMPL(m_cond, m_msg, ...) \
    if (unlikely(static_cast<bool>(m_cond))) { \
        godot::String _f_msg = godot::vformat(m_msg); \
        \
        /* This handles the console printing and clickable C++ link */ \
        ::godot::_err_print_error(FUNCTION_STR, __FILE__, __LINE__, "RCLGD Error", _f_msg, true, false); \
        \
        godot::EngineDebugger *ed = godot::EngineDebugger::get_singleton(); \
        godot::ProjectSettings *_ps = godot::ProjectSettings::get_singleton(); \
        bool _break = !_ps || (bool)_ps->get_setting("rclgd/debug/break_on_error", true); \
        if (_break && ed && ed->is_active()) { \
            /* Triggers the pause. We grab Language 0 (GDScript) from the Engine singleton. */ \
            ed->script_debug(godot::Engine::get_singleton()->get_script_language(0), true, true); \
        } \
        return __VA_ARGS__; \
    } else ((void)0)

/* --- PUBLIC API --- */

#define RCLGD_FAIL_MSG(m_msg) _RCLGD_FAIL_IMPL(true, m_msg)
#define RCLGD_FAIL_V_MSG(m_retval, m_msg) _RCLGD_FAIL_IMPL(true, m_msg, m_retval)

#define RCLGD_FAIL_COND_MSG(m_cond, m_msg) _RCLGD_FAIL_IMPL(m_cond, m_msg)
#define RCLGD_FAIL_COND_V_MSG(m_cond, m_retval, m_msg) _RCLGD_FAIL_IMPL(m_cond, m_msg, m_retval)

#define RCLGD_FAIL_NULL_V_MSG(m_param, m_retval, m_msg) _RCLGD_FAIL_IMPL((m_param == nullptr), m_msg, m_retval)

#endif // RCLGD_MACROS_HPP