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

    //Overrides for native Editor support
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

void RosMsg::gen_editor_support(const String &p_type, const String &p_dest_folder) {
    HashSet<String> processed; // Local to this call stack
    _gen_recursive(p_type, p_dest_folder, processed);
    // 'processed' is automatically destroyed here when the generation finishes.
}

void RosMsg::_gen_recursive(const String &p_type, const String &p_dest_folder, HashSet<String> &p_processed) {
    String ros_type = p_type.replace("::", "/");

    if (p_processed.has(ros_type)) return;
    p_processed.insert(ros_type);

    Ref<RosMsg> msg = RosMsg::from_type(ros_type);
    if (msg.is_null()) return;

    String class_name = msg->get_type_name();
    String code = "extends RosMsg\nclass_name " + class_name + "\n\n";

    code += "func _init():\n\tinit(\"" + ros_type + "\")\n\n";

    for (const KeyValue<StringName, Variant> &E : msg->members_) {
        Variant v = E.value;
        String field = String(E.key);
        String type_hint;

        // Type Hinting Logic
        if (v.get_type() == Variant::OBJECT) {
            Ref<RosMsg> nested = Object::cast_to<RosMsg>(v);
            type_hint = nested->get_type_name();
            _gen_recursive(nested->get_ros_interface_name(), p_dest_folder, p_processed);
        } 
        else if (v.get_type() == Variant::ARRAY && !((Array)v).is_empty() && ((Array)v)[0].get_type() == Variant::OBJECT) {
            Ref<RosMsg> entry = Object::cast_to<RosMsg>(((Array)v)[0]);
            type_hint = "Array[" + entry->get_type_name() + "]";
            _gen_recursive(entry->get_ros_interface_name(), p_dest_folder, p_processed);
        } 
        else {
            type_hint = Variant::get_type_name(v.get_type());
        }

        // THE BYPASS REDIRECTION
        // This makes 'var data' a virtual property that redirects to C++
        code += "var " + field + " : " + type_hint + ":\n";
        code += "\tget: return get_member(&\"" + field + "\")\n";
        code += "\tset(v): set_member(&\"" + field + "\", v)\n\n";
    }

    DirAccess::make_dir_recursive_absolute(p_dest_folder);
    Ref<FileAccess> f = FileAccess::open(p_dest_folder.path_join(class_name + ".gd"), FileAccess::WRITE);
    if (f.is_valid()) f->store_string(code);
}

// Direct access to the cache
Variant RosMsg::get_member(const StringName &p_name) const {
    if (members_.has(p_name)) {
        return members_[p_name];
    }
    return Variant();
}

// Redirects directly to your internal sync logic
void RosMsg::set_member(const StringName &p_name, const Variant &p_value) {
    this->_set(p_name, p_value);
}


void RosMsg::init_babel(const CompoundMessage::SharedPtr msg){

    if (!msg) {
        UtilityFunctions::push_error("Cant init babel without babel");
        return;
    }

    //Set our internal babel
    msg_ = msg;

    members_.clear(); // Safety clear

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
void RosMsg::init(const String &ros_type_name) {
    // If msg_ is already set, we've already initialized this instance.
    if (msg_) {
        // Soft error so it doesn't crash the editor
        UtilityFunctions::push_warning("RosMsg already initialized: " + get_type_name());
        return;
    }

    // Create the Babel Fish message
    CompoundMessage::SharedPtr msg = rclgd::get_singleton()->get_fish().create_message_shared(ros_type_name.utf8().get_data());
    
    if (!msg) {
        UtilityFunctions::push_error("Failed to get Fish Type Support for: " + ros_type_name);
        return;
    }

    // Build the Godot-facing member cache
    init_babel(msg);
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

bool RosMsg::_set(const StringName &p_name, const Variant &p_value)
{
    if (!members_.has(p_name) || !msg_)
        return false;

    std::string key = String(p_name).utf8().get_data();
    if (!msg_->containsKey(key))
        return false;

    try
    {
        // Get the ROS member
        Message &ros_member = (*msg_)[key];

        // MAGIC: The macro looks at the ROS type of the member,
        // and Godot2Ros::call extracts the data from the Variant correctly.
        RBF2_TEMPLATE_CALL(Godot2Ros::call, ros_member.type(), p_value, ros_member);

        // Update cache
        members_[p_name] = p_value;
        return true;
    }
    catch (const std::exception &e)
    {
        UtilityFunctions::printerr("ROS Sync Error: ", e.what());
        return false;
    }
}