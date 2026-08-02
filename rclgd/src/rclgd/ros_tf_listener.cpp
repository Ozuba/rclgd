#include "ros_tf_listener.hpp"
#include "ros_msg.hpp"
#include "utils/ros_tf_utils.hpp"

#include <tf2/buffer_core.hpp>

void RosTfListener::_bind_methods() {
    // One lookup, with the stamp and the timeout both optional: two arguments
    // for "where is it now", three for "where was it when this message was
    // taken".
    ClassDB::bind_method(D_METHOD("lookup_transform", "target_frame", "source_frame",
                                  "time", "timeout_sec"),
                         &RosTfListener::lookup_transform, DEFVAL(Variant()), DEFVAL(0.0));
    ClassDB::bind_method(D_METHOD("can_transform", "target_frame", "source_frame", "time"),
                         &RosTfListener::can_transform, DEFVAL(Variant()));
    ClassDB::bind_method(D_METHOD("lookup_transform_full", "target_frame", "target_time", "source_frame",
                                  "source_time", "fixed_frame", "timeout_sec"),
                         &RosTfListener::lookup_transform_full, DEFVAL(0.0));

    ClassDB::bind_method(D_METHOD("get_frame_names"), &RosTfListener::get_frame_names);
    ClassDB::bind_method(D_METHOD("frame_exists", "frame"), &RosTfListener::frame_exists);
    ClassDB::bind_method(D_METHOD("get_frame_parent", "frame"), &RosTfListener::get_frame_parent);
    ClassDB::bind_method(D_METHOD("get_frame_latest_time", "frame"), &RosTfListener::get_frame_latest_time);
    ClassDB::bind_method(D_METHOD("all_frames_as_yaml"), &RosTfListener::all_frames_as_yaml);

    ClassDB::bind_method(D_METHOD("get_last_error"), &RosTfListener::get_last_error);
    ClassDB::bind_method(D_METHOD("clear"), &RosTfListener::clear);
    ClassDB::bind_method(D_METHOD("get_cache_time"), &RosTfListener::get_cache_time);
}

void RosTfListener::setup(std::shared_ptr<rclcpp::Node> p_node, double p_cache_time_sec) {
    node_ = p_node;
    if (p_cache_time_sec <= 0.0)
        p_cache_time_sec = 10.0; // tf2's own default
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock(),
                                                   tf2::durationFromSec(p_cache_time_sec));
    // Subscribe through the owning node (spun by the global executor) instead
    // of letting TransformListener spawn its own internal node and thread.
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, false);
}

tf2::TimePoint RosTfListener::_to_time_point(const Variant &p_time) {
    switch (p_time.get_type()) {
    case Variant::NIL:
        // "Latest available" — the same behaviour as lookup_transform().
        return tf2::TimePointZero;
    case Variant::INT:
    case Variant::FLOAT:
        return tf2::timeFromSec((double)p_time);
    case Variant::OBJECT: {
        // builtin_interfaces/msg/Time, e.g. straight out of msg.header.stamp.
        Ref<RosMsg> stamp = p_time;
        if (stamp.is_valid()) {
            int64_t sec = stamp->get_member("sec");
            int64_t nanosec = stamp->get_member("nanosec");
            // A zero stamp means "latest" in ROS, and an exact lookup at the
            // epoch would fail for every frame.
            if (sec == 0 && nanosec == 0)
                return tf2::TimePointZero;
            return tf2::TimePoint(std::chrono::seconds(sec) + std::chrono::nanoseconds(nanosec));
        }
        return tf2::TimePointZero;
    }
    default:
        return tf2::TimePointZero;
    }
}

Variant RosTfListener::lookup_transform(const String &p_target_frame, const String &p_source_frame,
                                        const Variant &p_time, double p_timeout_sec) {
    if (!tf_buffer_ || !node_ || !rclcpp::ok()) {
        last_error_ = "ROS context is not running";
        return Variant();
    }

    try {
        // 1. Resolve frames (expands ~ to the node namespace)
        std::string target = RclgdUtils::resolve_frame(node_, p_target_frame);
        std::string source = RclgdUtils::resolve_frame(node_, p_source_frame);

        // 2. Look the transform up at the requested stamp
        auto t = tf_buffer_->lookupTransform(target, source, _to_time_point(p_time),
                                             tf2::durationFromSec(p_timeout_sec));

        // 3. Map ROS convention to Godot convention (shared helper)
        last_error_ = String();
        return RclgdUtils::ros_to_godot_transform(t.transform);

    } catch (const tf2::TransformException &e) {
        // Lookup failures are routine while frames are being published; return
        // null so the caller can tell, without breaking into the debugger. The
        // reason stays available through get_last_error().
        last_error_ = String(e.what());
        return Variant();
    }
}

bool RosTfListener::can_transform(const String &p_target_frame, const String &p_source_frame,
                                  const Variant &p_time) const {
    if (!tf_buffer_ || !node_ || !rclcpp::ok()) return false;
    std::string target = RclgdUtils::resolve_frame(node_, p_target_frame);
    std::string source = RclgdUtils::resolve_frame(node_, p_source_frame);
    std::string err;
    bool ok = tf_buffer_->canTransform(target, source, _to_time_point(p_time),
                                       tf2::durationFromSec(0.0), &err);
    if (!ok)
        last_error_ = String(err.c_str());
    return ok;
}

Variant RosTfListener::lookup_transform_full(const String &p_target_frame, const Variant &p_target_time,
                                             const String &p_source_frame, const Variant &p_source_time,
                                             const String &p_fixed_frame, double p_timeout_sec) {
    if (!tf_buffer_ || !node_ || !rclcpp::ok()) {
        last_error_ = "ROS context is not running";
        return Variant();
    }

    try {
        std::string target = RclgdUtils::resolve_frame(node_, p_target_frame);
        std::string source = RclgdUtils::resolve_frame(node_, p_source_frame);
        std::string fixed = RclgdUtils::resolve_frame(node_, p_fixed_frame);

        auto t = tf_buffer_->lookupTransform(target, _to_time_point(p_target_time),
                                             source, _to_time_point(p_source_time),
                                             fixed, tf2::durationFromSec(p_timeout_sec));
        last_error_ = String();
        return RclgdUtils::ros_to_godot_transform(t.transform);

    } catch (const tf2::TransformException &e) {
        last_error_ = String(e.what());
        return Variant();
    }
}

PackedStringArray RosTfListener::get_frame_names() const {
    PackedStringArray out;
    if (!tf_buffer_) return out;
    for (const std::string &name : tf_buffer_->getAllFrameNames())
        out.push_back(String(name.c_str()));
    return out;
}

bool RosTfListener::frame_exists(const String &p_frame) const {
    if (!tf_buffer_ || !node_) return false;
    return tf_buffer_->_frameExists(RclgdUtils::resolve_frame(node_, p_frame));
}

String RosTfListener::get_frame_parent(const String &p_frame) const {
    if (!tf_buffer_ || !node_) return String();
    std::string parent;
    // Roots (and unknown frames) report no parent; that is not an error, so it
    // does not touch last_error_.
    if (!tf_buffer_->_getParent(RclgdUtils::resolve_frame(node_, p_frame), tf2::TimePointZero, parent))
        return String();
    return String(parent.c_str());
}

double RosTfListener::get_frame_latest_time(const String &p_frame) const {
    if (!tf_buffer_ || !node_) return 0.0;
    std::string frame = RclgdUtils::resolve_frame(node_, p_frame);
    tf2::CompactFrameID id = tf_buffer_->_lookupFrameNumber(frame);
    if (id == 0) return 0.0; // unknown frame

    // Asking for the latest time common to a frame and itself is how tf2
    // reports that frame's newest stored stamp.
    tf2::TimePoint latest;
    if (tf_buffer_->_getLatestCommonTime(id, id, latest, nullptr) != tf2::TF2Error::TF2_NO_ERROR)
        return 0.0;
    return tf2::timeToSec(latest);
}

String RosTfListener::all_frames_as_yaml() const {
    if (!tf_buffer_) return String();
    return String(tf_buffer_->allFramesAsYAML().c_str());
}

void RosTfListener::clear() {
    if (!tf_buffer_) return;
    tf_buffer_->clear();
    last_error_ = String();
}

double RosTfListener::get_cache_time() const {
    if (!tf_buffer_) return 0.0;
    return tf2::durationToSec(tf_buffer_->getCacheLength());
}
