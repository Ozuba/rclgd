#include "ros_msg.hpp"
#include "utils/ros_type_utils.hpp"
#include <sstream>

RosMsg::RosMsg()
{
}

RosMsg::~RosMsg()
{
}

void RosMsg::_bind_methods()
{
    // Bind Type Factory
    ClassDB::bind_static_method(
        "RosMsg",                               // The class receiving the method
        D_METHOD("from_type", "ros_type_name"), // GDScript signature
        &RosMsg::from_type                      // C++ function pointer
    );
    //Bind str representation
    ClassDB::bind_method(D_METHOD("_to_string"), &RosMsg::_to_string);
}

Ref<RosMsg> RosMsg::from_type(const String &ros_typename)
{
    RosMsg *raw_ptr = memnew(RosMsg);
    Ref<RosMsg> ref(raw_ptr);
    // Initialize fish type object and make RosMsg point to this
    ref->init(rclgd::get_singleton()->get_fish().create_message_shared(ros_typename.utf8().get_data()));

    // Build property list
    return ref;
}

void RosMsg::init(ros_babel_fish::CompoundMessage::SharedPtr p_msg)
{
    msg_ = p_msg;
    members_.clear(); // Safety clear

    if (!msg_)
        return;
    // Build our member map
    //  1. Get the list of names and the list of SharedPtrs
    std::vector<std::string> keys = msg_->keys();
    std::vector<ros_babel_fish::Message::SharedPtr> msg_values = msg_->values();

    // 2. Iterate using an index so we can match key to value
    for (size_t i = 0; i < keys.size(); ++i)
    {
        StringName member_name = String(keys[i].c_str());
        Variant value;

        // This is the correct way to get the pre-managed SharedPtr
        ros_babel_fish::Message::SharedPtr member_ptr = msg_values[i];

        // Convert ROS data to Godot Variant
        RBF2_TEMPLATE_CALL(Ros2Godot::call, member_ptr->type(), value, member_ptr);

        // Store in your HashMap
        members_[member_name] = value;
    }
}

String RosMsg::_to_string() const {
    // Start the string representation
    String out = "RosMsg {\n";

    // Iterate over the Godot-converted members we already have in our cache
    for (const KeyValue<StringName, Variant> &E : members_) {
        String key = String(E.key);
        
        // Variant::stringify() or a simple cast to String works here.
        // If the member is another RosMsg, Godot will call its _to_string() automatically.
        String value_str = E.value.stringify();
        
        // Add indentation for a clean look
        out += "  \"" + key + "\": " + value_str + "\n";
    }

    out += "}";
    return out;
}

// Overloaded Accerors

void RosMsg::_get_property_list(List<PropertyInfo> *p_list) const
{
    // Iterate over our converted Godot members
    for (const KeyValue<StringName, Variant> &E : members_)
    {
        // Create a PropertyInfo based on the actual Variant type of the member
        // This tells the Inspector if it's a float, int, RosMsg (Object), etc.
        p_list->push_back(PropertyInfo(E.value.get_type(), E.key));
    }
}

bool RosMsg::_get(const StringName &p_name, Variant &r_ret) const
{
    if (members_.has(p_name))
    {
        r_ret = members_[p_name];
        return true;
    }
    return false;
}


bool RosMsg::_set(const StringName &p_name, const Variant &p_value) {
    if (!members_.has(p_name) || !msg_) return false;

    std::string key = String(p_name).utf8().get_data();
    if (!msg_->containsKey(key)) return false;

    try {
        // Get the ROS member
        Message &ros_member = (*msg_)[key];

        // MAGIC: The macro looks at the ROS type of the member, 
        // and Godot2Ros::call extracts the data from the Variant correctly.
        RBF2_TEMPLATE_CALL(Godot2Ros::call, ros_member.type(), p_value, ros_member);

        // Update cache
        members_[p_name] = p_value;
        return true;
    } 
    catch (const std::exception &e) {
        UtilityFunctions::printerr("ROS Sync Error: ", e.what());
        return false;
    }
}