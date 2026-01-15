#ifndef ROS_QOS_PROFILE_H
#define ROS_QOS_PROFILE_H

#include <godot_cpp/classes/resource.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <rclcpp/qos.hpp>

namespace godot {

class RosQoS : public Resource {
    GDCLASS(RosQoS, Resource)

public:
    enum Reliability { RELIABLE, BEST_EFFORT };
    enum Durability { VOLATILE, TRANSIENT_LOCAL };
    enum History { KEEP_LAST, KEEP_ALL };
    enum Liveliness { AUTOMATIC, MANUAL_BY_TOPIC, SYSTEM_DEFAULT };

private:
    History history = KEEP_LAST;
    int depth = 10;
    Reliability reliability = RELIABLE;
    Durability durability = VOLATILE;
    float deadline = 0.0f;
    float lifespan = 0.0f;
    float liveliness_lease_duration = 0.0f;
    Liveliness liveliness = SYSTEM_DEFAULT;

protected:
    static void _bind_methods();

public:
    RosQoS() = default;
    ~RosQoS() = default;

    // Core conversion function
    rclcpp::QoS get_qos() const;

    // Getters and Setters for Godot
    void set_history(int p_val) { history = (History)p_val; emit_changed(); }
    int get_history() const { return (int)history; }

    void set_depth(int p_val) { depth = p_val; emit_changed(); }
    int get_depth() const { return depth; }

    void set_reliability(int p_val) { reliability = (Reliability)p_val; emit_changed(); }
    int get_reliability() const { return (int)reliability; }

    void set_durability(int p_val) { durability = (Durability)p_val; emit_changed(); }
    int get_durability() const { return (int)durability; }

    void set_deadline(float p_val) { deadline = p_val; emit_changed(); }
    float get_deadline() const { return deadline; }

    void set_lifespan(float p_val) { lifespan = p_val; emit_changed(); }
    float get_lifespan() const { return lifespan; }

    void set_liveliness(int p_val) { liveliness = (Liveliness)p_val; emit_changed(); }
    int get_liveliness() const { return (int)liveliness; }

    void set_lease_duration(float p_val) { liveliness_lease_duration = p_val; emit_changed(); }
    float get_lease_duration() const { return liveliness_lease_duration; }
    
};

} // namespace godot

VARIANT_ENUM_CAST(RosQoS::Reliability);
VARIANT_ENUM_CAST(RosQoS::Durability);
VARIANT_ENUM_CAST(RosQoS::History);
VARIANT_ENUM_CAST(RosQoS::Liveliness);
#endif