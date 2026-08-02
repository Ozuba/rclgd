#ifndef ROS_TF_LISTENER_HPP
#define ROS_TF_LISTENER_HPP

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/variant.hpp>
#include <godot_cpp/variant/packed_string_array.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "rclgd.hpp"



using namespace godot;

class RosTfListener : public RefCounted {
    GDCLASS(RosTfListener, RefCounted);

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Text of the most recent failed lookup. Viewers surface this verbatim as
    // the reason a display cannot be drawn, so it has to outlive the call that
    // produced it.
    mutable String last_error_;

    // Interpret the `time` argument the lookups share:
    //   null      -> tf2::TimePointZero ("latest available")
    //   float/int -> seconds since epoch
    //   RosMsg    -> builtin_interfaces/msg/Time (sec + nanosec)
    static tf2::TimePoint _to_time_point(const Variant &p_time);

protected:
    static void _bind_methods();

public:
    RosTfListener() = default;
    ~RosTfListener() = default;

    void setup(std::shared_ptr<rclcpp::Node> p_node, double p_cache_time_sec);

    // Returns a Transform3D on success or null on failure, so callers can
    // distinguish "lookup failed" from an actual identity transform.
    //
    // Omitting p_time gives the newest transform available. Passing a message's
    // header.stamp instead resolves it at the moment the data was captured,
    // which is what keeps sensor readings pinned to where the robot actually
    // was when it took them.
    Variant lookup_transform(const String &p_target_frame, const String &p_source_frame,
                             const Variant &p_time = Variant(), double p_timeout_sec = 0.0);
    bool can_transform(const String &p_target_frame, const String &p_source_frame,
                       const Variant &p_time = Variant()) const;

    // Advanced ("time travel") lookup: where source_frame was at source_time,
    // expressed in target_frame at target_time, bridged through fixed_frame.
    Variant lookup_transform_full(const String &p_target_frame, const Variant &p_target_time,
                                  const String &p_source_frame, const Variant &p_source_time,
                                  const String &p_fixed_frame, double p_timeout_sec = 0.0);

    // --- Frame graph introspection ---
    // Which frames exist, how they nest, and how fresh each one is.
    PackedStringArray get_frame_names() const;
    bool frame_exists(const String &p_frame) const;
    // Empty string when the frame is a tree root or is not in the buffer.
    String get_frame_parent(const String &p_frame) const;
    // Stamp of the newest transform stored for the frame, in seconds; 0.0 when
    // the frame is unknown or has never been updated.
    double get_frame_latest_time(const String &p_frame) const;
    // The tf2 debug dump, same content as `ros2 run tf2_tools view_frames`.
    String all_frames_as_yaml() const;

    // Reason the last lookup failed ("" once one succeeds).
    String get_last_error() const { return last_error_; }
    // Drop every stored transform, e.g. when a viewer is reset or /clock jumps
    // backwards.
    void clear();
    double get_cache_time() const;
};

#endif // ROS_TF_LISTENER_HPP
