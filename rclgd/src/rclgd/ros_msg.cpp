#include "ros_msg.hpp"
#include "ros_type_registry.hpp"
#include "utils/ros_type_utils.hpp"

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

    // Type initialization
    ClassDB::bind_method(
        D_METHOD("init", "ros_type_name"), // GDScript signature
        &RosMsg::init                      // C++ function pointer
    );

    // Bind str representation
    ClassDB::bind_method(D_METHOD("_to_string"), &RosMsg::_to_string);

    // Get type name
    ClassDB::bind_method(D_METHOD("get_type_name"), &RosMsg::get_type_name);

    // Overrides for native Editor support
    ClassDB::bind_method(D_METHOD("get_member", "p_name"), &RosMsg::get_member);
    ClassDB::bind_method(D_METHOD("set_member", "p_name", "p_value"), &RosMsg::set_member);

    ClassDB::bind_method(D_METHOD("get_ros_interface_name"), &RosMsg::get_ros_interface_name);
}

// Factory Creator Method
Ref<RosMsg> RosMsg::from_type(const String &ros_type_name)
{
    // Goes through wrap() so a registered type comes back as its native typed
    // class. If type support lookup fails, return null instead of a hollow
    // message so callers can detect the failure with is_null()/== null.
    CompoundMessage::SharedPtr msg = _create_babel(ros_type_name);
    return msg ? wrap(msg) : Ref<RosMsg>();
}

CompoundMessage::SharedPtr RosMsg::_create_babel(const String &p_ros_type)
{
    try
    {
        return rclgd::get_singleton()->get_fish().create_message_shared(p_ros_type.utf8().get_data());
    }
    catch (const std::exception &e)
    {
        RCLGD_FAIL_V_MSG(nullptr, vformat("RCLGD: Failed to create ROS message '%s': %s", p_ros_type, e.what()));
    }
}

Ref<RosMsg> RosMsg::wrap(const CompoundMessage::SharedPtr &p_msg)
{
    if (p_msg)
    {
        if (const RosTypeRegistry::TypeInfo *type = RosTypeRegistry::find(p_msg->datatype()))
            return RosTypeRegistry::instantiate(type, p_msg);
    }
    Ref<RosMsg> ref;
    ref.instantiate();
    if (p_msg)
        ref->init_babel(p_msg);
    return ref;
}

void RosMsg::init_babel(const CompoundMessage::SharedPtr msg)
{
    ERR_FAIL_NULL_MSG(msg, "RCLGD: Cannot init_babel with a null shared_ptr.");
    msg_ = msg;
    members_.clear();

    // Members are NOT converted here — that happens lazily on first access in
    // get_member(); nested messages are wrapped only when their field is
    // actually read.
}

Variant RosMsg::_convert_member(const StringName &p_name) const
{
    if (!msg_)
        return Variant();

    std::string key = String(p_name).utf8().get_data();
    if (!msg_->containsKey(key))
        return Variant();

    ros_babel_fish::Message &member = (*msg_)[key];
    Variant value;
    // Conversion runs on the executor thread too (incoming messages), where an
    // escaping BabelFish exception would kill the whole engine — degrade to a
    // null Variant instead.
    try
    {
        // 'member_ptr' aliases 'member' but shares the ref-count with 'msg_', so
        // a nested message wrapped from it writes into our buffer.
        ros_babel_fish::Message::SharedPtr member_ptr(msg_, &member);
        RBF2_TEMPLATE_CALL(Ros2Godot::call, member.type(), value, member_ptr);
    }
    catch (const std::exception &e)
    {
        RCLGD_FAIL_V_MSG(Variant(), vformat("RCLGD: Failed to convert field '%s': %s", String(p_name), e.what()));
    }
    return value;
}

void RosMsg::init(const String &ros_type_name)
{
    // If msg_ is already set, we've already initialized this instance (e.g. a
    // native typed class, built with its message): treat it as a no-op.
    if (msg_)
    {
        return;
    }

    if (CompoundMessage::SharedPtr msg = _create_babel(ros_type_name))
        init_babel(msg);
}

String RosMsg::class_name_for_datatype(const String &p_ros_datatype)
{
    // "std_msgs::msg::Header" -> "RosStdMsgsHeader"
    return "Ros" + p_ros_datatype.replace("::msg::", " ").replace("::", " ").to_pascal_case();
}

String RosMsg::get_type_name() const
{
    if (!msg_)
        return "RosUnknownType";
    // We use the C++ Datatype "std_msgs::msg::Header" to build the class name
    return class_name_for_datatype(String(msg_->datatype().c_str()));
}

// For the ROS Factory: "std_msgs/msg/Header"
String RosMsg::get_ros_interface_name() const
{
    if (!msg_)
        return "";
    // We use the Name "std_msgs/msg/Header" for the factory and recursion
    // This matches BabelFish's getMessageName() logic you found.
    return String(msg_->name().c_str());
}

String RosMsg::_to_string() const
{
    // Start the string representation
    String out = get_type_name() + " {\n";

    if (msg_)
    {
        // Walk the schema (not the cache) so every field is shown; get_member
        // lazily converts each one. Stringifying is a debug path, not hot.
        for (const std::string &key_str : msg_->keys())
        {
            StringName key(String(key_str.c_str()));
            out += "  \"" + String(key) + "\": " + get_member(key).stringify() + "\n";
        }
    }

    out += "}";
    return out;
}

// Overloaded Accessors

void RosMsg::_get_property_list(List<PropertyInfo> *p_list) const
{
    // A native typed class already lists its fields as registered ClassDB
    // properties; reporting them again would double them in the inspector.
    if (!msg_ || typed_)
    {
        return;
    }

    // Report the schema's members; convert lazily to learn each Variant type
    // (inspector path only, not the receive hot path).
    for (const std::string &key_str : msg_->keys())
    {
        StringName key(String(key_str.c_str()));
        p_list->push_back(PropertyInfo(get_member(key).get_type(), key));
    }
}

bool RosMsg::_get(const StringName &p_name, Variant &r_ret) const
{
    if (!msg_)
        return false;
    // Fast path: already converted. Avoids a schema key scan on repeated reads.
    HashMap<StringName, Variant>::Iterator it = members_.find(p_name);
    if (it)
    {
        r_ret = it->value;
        return true;
    }
    // Cold path: membership is decided by the schema, then convert + cache.
    if (!msg_->containsKey(String(p_name).utf8().get_data()))
        return false;
    r_ret = get_member(p_name);
    return true;
}

bool RosMsg::_set(const StringName &p_name, const Variant &p_value)
{
    if (!msg_)
        return false;

    std::string key = String(p_name).utf8().get_data();
    // Membership is decided by the schema; the cache may not be populated yet.
    if (!msg_->containsKey(key))
        return false;
    try
    {
        ros_babel_fish::Message &ros_member = (*msg_)[key];

        // Error out if assigning a non RosMsg to a RosMsg type
        if (ros_member.type() == MessageTypes::Compound && Ref<RosMsg>(p_value).is_null())
            return false;

        // Sync Variant -> ROS Buffer
        RBF2_TEMPLATE_CALL(Godot2Ros::call, ros_member.type(), p_value, ros_member);

        // Update Cache.
        // For compounds (and arrays of compounds) the data was *copied* into
        // this message's buffer, so caching p_value would leave the cache
        // wrapping the assigned object's own buffer: later nested writes
        // (e.g. msg.header.stamp = x) would never reach this message.
        // Re-wrap the member so the cache aliases our buffer instead.
        bool is_compound = ros_member.type() == MessageTypes::Compound;
        bool is_compound_array = ros_member.type() == MessageTypes::Array &&
                                 ros_member.as<ArrayMessageBase>().elementType() == MessageTypes::Compound;
        if (is_compound || is_compound_array)
        {
            Variant aliased;
            ros_babel_fish::Message::SharedPtr member_ptr(msg_, &ros_member);
            RBF2_TEMPLATE_CALL(Ros2Godot::call, ros_member.type(), aliased, member_ptr);
            members_[p_name] = aliased;
        }
        else
        {
            members_[p_name] = p_value;
        }
        return true;
    }
    catch (const std::exception &e)
    {
        ERR_PRINT_ED(vformat("RCLGD: Error syncing field '%s' to ROS buffer: %s", String(p_name), e.what()));
        return false;
    }
}

// Lazy accessor: convert the member on first access, then serve from cache.
Variant RosMsg::get_member(const StringName &p_name) const
{
    HashMap<StringName, Variant>::Iterator it = members_.find(p_name);
    if (it)
        return it->value;

    Variant value = _convert_member(p_name);
    members_[p_name] = value;
    return value;
}

void RosMsg::set_member(const StringName &p_name, const Variant &p_value)
{
    // _set syncs the ROS buffer and refreshes the cache.
    this->_set(p_name, p_value);
}