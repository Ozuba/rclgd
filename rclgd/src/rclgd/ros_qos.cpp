#include "ros_qos.hpp"

using namespace godot;

void RosQoS::_bind_methods() {
    // Register Enums
    BIND_ENUM_CONSTANT(RELIABLE);
    BIND_ENUM_CONSTANT(BEST_EFFORT);
    BIND_ENUM_CONSTANT(VOLATILE);
    BIND_ENUM_CONSTANT(TRANSIENT_LOCAL);
    BIND_ENUM_CONSTANT(KEEP_LAST);
    BIND_ENUM_CONSTANT(KEEP_ALL);
    BIND_ENUM_CONSTANT(AUTOMATIC);
    BIND_ENUM_CONSTANT(MANUAL_BY_TOPIC);
    BIND_ENUM_CONSTANT(SYSTEM_DEFAULT);

    // Bind Core Policies
    ClassDB::bind_method(D_METHOD("get_history"), &RosQoS::get_history);
    ClassDB::bind_method(D_METHOD("set_history", "p_val"), &RosQoS::set_history);
    ADD_PROPERTY(PropertyInfo(Variant::INT, "history", PROPERTY_HINT_ENUM, "Keep Last,Keep All"), "set_history", "get_history");

    ClassDB::bind_method(D_METHOD("get_depth"), &RosQoS::get_depth);
    ClassDB::bind_method(D_METHOD("set_depth", "p_val"), &RosQoS::set_depth);
    ADD_PROPERTY(PropertyInfo(Variant::INT, "depth"), "set_depth", "get_depth");

    ClassDB::bind_method(D_METHOD("get_reliability"), &RosQoS::get_reliability);
    ClassDB::bind_method(D_METHOD("set_reliability", "p_val"), &RosQoS::set_reliability);
    ADD_PROPERTY(PropertyInfo(Variant::INT, "reliability", PROPERTY_HINT_ENUM, "Reliable,Best Effort"), "set_reliability", "get_reliability");

    ClassDB::bind_method(D_METHOD("get_durability"), &RosQoS::get_durability);
    ClassDB::bind_method(D_METHOD("set_durability", "p_val"), &RosQoS::set_durability);
    ADD_PROPERTY(PropertyInfo(Variant::INT, "durability", PROPERTY_HINT_ENUM, "Volatile,Transient Local"), "set_durability", "get_durability");

    // Bind Advanced Timings
    ClassDB::bind_method(D_METHOD("get_deadline"), &RosQoS::get_deadline);
    ClassDB::bind_method(D_METHOD("set_deadline", "p_val"), &RosQoS::set_deadline);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "deadline_seconds"), "set_deadline", "get_deadline");

    ClassDB::bind_method(D_METHOD("get_lifespan"), &RosQoS::get_lifespan);
    ClassDB::bind_method(D_METHOD("set_lifespan", "p_val"), &RosQoS::set_lifespan);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "lifespan_seconds"), "set_lifespan", "get_lifespan");
}

rclcpp::QoS RosQoS::get_qos() const {
    // 1. Initialize History and Depth
    rclcpp::QoS qos = (history == KEEP_ALL) ? rclcpp::QoS(rclcpp::KeepAll()) : rclcpp::QoS(depth);

    // 2. Map Reliability
    if (reliability == BEST_EFFORT) qos.best_effort();
    else qos.reliable();

    // 3. Map Durability
    if (durability == TRANSIENT_LOCAL) qos.transient_local();
    else qos.durability_volatile();

    // 4. Map Advanced Durations
    if (deadline > 0.001f) qos.deadline(rclcpp::Duration::from_seconds(deadline));
    if (lifespan > 0.001f) qos.lifespan(rclcpp::Duration::from_seconds(lifespan));
    if (liveliness_lease_duration > 0.001f) qos.liveliness_lease_duration(rclcpp::Duration::from_seconds(liveliness_lease_duration));

    // 5. Map Liveliness
    switch (liveliness) {
        case AUTOMATIC: qos.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC); break;
        case MANUAL_BY_TOPIC: qos.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC); break;
        default: break; // System default
    }

    return qos;
}