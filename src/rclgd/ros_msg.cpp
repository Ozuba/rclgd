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

    // Bind type support generation
    ClassDB::bind_static_method(
        "RosMsg",                                                  // The class receiving the method
        D_METHOD("gen_editor_support", "p_type", "p_dest_folder"), // GDScript signature
        &RosMsg::gen_editor_support                                // C++ function pointer
    );

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
}

// Factory Creator Method
Ref<RosMsg> RosMsg::from_type(const String &ros_type_name)
{
    Ref<RosMsg> ref;
    ref.instantiate();
    ref->init(ros_type_name);
    return ref;
}

void RosMsg::gen_editor_support(const String &p_type, const String &p_dest_folder)
{
    HashSet<String> processed; // Local to this call stack
    _gen_recursive(p_type, p_dest_folder, processed);
    // 'processed' is automatically destroyed here when the generation finishes.
}

void RosMsg::_gen_recursive(const String &p_type, const String &p_dest_folder, HashSet<String> &p_processed)
{
    String ros_type = p_type.replace("::", "/");
    if (p_processed.has(ros_type))
        return;
    p_processed.insert(ros_type);

    Ref<RosMsg> msg = RosMsg::from_type(ros_type);
    if (msg.is_null())
        return;

    String class_name = msg->get_type_name();
    String code = "extends RosMsg\nclass_name " + class_name + "\n\n";
    code += "func _init():\n\tinit(\"" + ros_type + "\")\n\n";

    for (const KeyValue<StringName, Variant> &E : msg->members_)
    {
        String field = String(E.key);
        String type_hint;
        bool needs_cast = false;

        if (E.value.get_type() == Variant::OBJECT)
        {
            Ref<RosMsg> nested = Object::cast_to<RosMsg>(E.value);
            type_hint = nested->get_type_name();
            needs_cast = true; // Object types need the 'as' keyword
            _gen_recursive(nested->get_ros_interface_name(), p_dest_folder, p_processed);
        }
        else
        {
            type_hint = Variant::get_type_name(E.value.get_type());
        }

        code += "var " + field + " : " + type_hint + ":\n";
        code += "\tget: return get_member(&\"" + field + "\")" + (needs_cast ? " as RosMsg" : "") + "\n";
        code += "\tset(v): set_member(&\"" + field + "\", v)\n\n";
    }

    DirAccess::make_dir_recursive_absolute(p_dest_folder);
    Ref<FileAccess> f = FileAccess::open(p_dest_folder.path_join(class_name + ".gd"), FileAccess::WRITE);
    if (f.is_valid())
        f->store_string(code);
}

void RosMsg::init_babel(const CompoundMessage::SharedPtr msg)
{
    if (!msg)
        return;
    msg_ = msg;
    members_.clear();

    for (const std::string &key_str : msg_->keys())
    {
        StringName member_name = String(key_str.c_str());
        ros_babel_fish::Message &member = (*msg_)[key_str];
        Variant value;

        if (member.type() == ros_babel_fish::MessageTypes::Compound)
        {
            // RECURSIVE BRANCH: Initialize nested message
            Ref<RosMsg> sub;
            sub.instantiate();

            // Aliasing: 'sub_ptr' points to 'member' but shares ref-count with 'msg_'
            auto sub_ptr = std::shared_ptr<ros_babel_fish::CompoundMessage>(
                msg_,
                &member.as<ros_babel_fish::CompoundMessage>());

            sub->init_babel(sub_ptr);
            value = sub;
        }
        else
        {
            // LEAF NODE: Primitive or PackedArray
            // Create a temporary shared_ptr alias to satisfy the Ros2Godot template
            ros_babel_fish::Message::SharedPtr member_ptr(msg_, &member);
            RBF2_TEMPLATE_CALL(Ros2Godot::call, member.type(), value, member_ptr);
        }

        // Store in cache for the GDScript getters
        members_[member_name] = value;
    }
}

void RosMsg::init(const String &ros_type_name)
{
    // If msg_ is already set, we've already initialized this instance.
    if (msg_)
    {
        // Soft error so it doesn't crash the editor
        UtilityFunctions::push_warning("RosMsg already initialized: " + get_type_name());
        return;
    }

    // Create the Babel Fish message
    try
    {
        CompoundMessage::SharedPtr msg = rclgd::get_singleton()->get_fish().create_message_shared(ros_type_name.utf8().get_data());
        // Build the Godot-facing member cache
        init_babel(msg);
    }
    catch (const std::exception &e)
    {
        UtilityFunctions::push_error("Failed to get Fish Type Support: ", String(e.what()));
        return;
    }
}

String RosMsg::get_type_name() const
{
    if (!msg_)
        return "RosUnknownType";
    // We use the C++ Datatype "std_msgs::msg::Header" to build the class name
    String raw = String(msg_->datatype().c_str());
    return "Ros" + raw.replace("::msg::", " ").replace("::", " ").to_pascal_case();
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

    // Iterate over the Godot-converted members we already have in our cache
    for (const KeyValue<StringName, Variant> &E : members_)
    {
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
    // If we have a script attached, let the script define the properties
    // to avoid double-entries in the inspector.
    if (get_script().get_type() != Variant::NIL)
    {
        return;
    }

    for (const KeyValue<StringName, Variant> &E : members_)
    {
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

bool RosMsg::_set(const StringName &p_name, const Variant &p_value)
{
    if (!msg_ || !members_.has(p_name))
        return false;

    std::string key = String(p_name).utf8().get_data();
    try
    {
        ros_babel_fish::Message &ros_member = (*msg_)[key];

        // Sync Variant -> ROS Buffer
        RBF2_TEMPLATE_CALL(Godot2Ros::call, ros_member.type(), p_value, ros_member);

        // Update Cache
        members_[p_name] = p_value;
        return true;
    }
    catch (const std::exception &e)
    {
        return false;
    }
}

// Direct access to the cache
Variant RosMsg::get_member(const StringName &p_name) const
{
    // Because of Deep Init, if it's not in the cache, it doesn't exist.
    if (members_.has(p_name))
    {
        return members_[p_name];
    }
    return Variant();
}

void RosMsg::set_member(const StringName &p_name, const Variant &p_value)
{
    // 1. Update the actual ROS2 data buffer (BabelFish)
    // Your existing _set handles the Variant -> ROS conversion
    if (this->_set(p_name, p_value))
    {
        // 2. Sync the cache so getters are immediate
        members_[p_name] = p_value;
    }
}