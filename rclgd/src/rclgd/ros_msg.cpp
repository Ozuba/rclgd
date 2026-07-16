#include "ros_msg.hpp"
#include "utils/ros_type_utils.hpp"
#include <godot_cpp/classes/project_settings.hpp>
#include <godot_cpp/classes/resource_loader.hpp>
#include <godot_cpp/classes/script.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/typed_array.hpp>
#include <mutex>
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

    ClassDB::bind_method(D_METHOD("get_ros_interface_name"), &RosMsg::get_ros_interface_name);
}

// Factory Creator Method
Ref<RosMsg> RosMsg::from_type(const String &ros_type_name)
{
    Ref<RosMsg> ref;
    ref.instantiate();
    ref->init(ros_type_name);
    // If type support lookup failed, return null instead of a hollow message
    // so callers can detect the failure with is_null()/== null.
    if (!ref->get_babel())
        return Ref<RosMsg>();
    return ref;
}

// Shadow-type registry. Kept at file scope (not function-local statics) so it can
// be cleared during extension de-initialization via clear_type_registry(). It must
// NOT hold its Ref<Script> values past engine teardown: a static Ref destructing at
// libc atexit unrefs a Script whose backing Godot already freed -> use-after-free
// crash on exit. So g_loaded is emptied in clear_type_registry() while the engine
// is still alive.
namespace
{
    std::mutex g_registry_mutex;
    HashMap<String, String> g_class_paths;     // class name -> .gd path (no Refs, safe)
    HashMap<String, Ref<Script>> g_loaded;     // class name -> loaded Script (cleared at deinit)
    bool g_registry_indexed = false;
}

Ref<Script> RosMsg::script_for_class(const String &p_class_name)
{
    // The class-name -> path index is built once from the project's global class
    // list (cheap, no resource loading); the Scripts themselves are loaded lazily
    // on first request and cached. Guarded with a mutex because messages are
    // wrapped on the ROS executor thread, not just the main thread.
    std::lock_guard<std::mutex> lock(g_registry_mutex);

    if (!g_registry_indexed)
    {
        TypedArray<Dictionary> classes = ProjectSettings::get_singleton()->get_global_class_list();
        for (int i = 0; i < classes.size(); ++i)
        {
            Dictionary d = classes[i];
            // Every generated shadow type derives directly from RosMsg.
            if (String(d.get("base", "")) != "RosMsg")
                continue;
            String name = d.get("class", "");
            String path = d.get("path", "");
            if (!name.is_empty() && !path.is_empty())
                g_class_paths[name] = path;
        }
        g_registry_indexed = true;
    }

    if (g_loaded.has(p_class_name))
        return g_loaded[p_class_name];
    if (!g_class_paths.has(p_class_name))
        return Ref<Script>();

    // Cache the result either way (valid or null) so we don't retry on every message.
    Ref<Script> scr = ResourceLoader::get_singleton()->load(g_class_paths[p_class_name]);
    g_loaded[p_class_name] = scr;
    return scr;
}

void RosMsg::clear_type_registry()
{
    // Called from extension de-init while the engine is still up. Drops the cached
    // Script Refs here so they are NOT unref'd later at atexit (when Godot's
    // ResourceDB is gone), which would crash. Safe to rebuild lazily afterwards.
    std::lock_guard<std::mutex> lock(g_registry_mutex);
    g_loaded.clear();
    g_class_paths.clear();
    g_registry_indexed = false;
}

void RosMsg::_apply_script(RosMsg *p_msg)
{
    if (!p_msg)
        return;
    // Already a scripted instance (e.g. built via `RosXxx.new()`): leave it alone.
    if (p_msg->get_script().get_type() != Variant::NIL)
        return;

    Ref<Script> scr = script_for_class(p_msg->get_type_name());
    if (scr.is_valid())
        p_msg->set_script(scr); // re-runs _init -> init(), which no-ops (msg_ is set)
}

void RosMsg::gen_editor_support(const String &p_type, const String &p_dest_folder)
{
    RCLGD_FAIL_COND_MSG(p_type.is_empty(), "RCLGD: Cannot generate support for empty type name.");
    RCLGD_FAIL_COND_MSG(p_dest_folder.is_empty(), "RCLGD: Dest folder path is empty.");
    
    HashSet<String> processed; // Local to this call stack
    // A C++ exception must not cross the GDScript boundary (it would
    // std::terminate the editor); fail this one type and let the caller
    // continue with the rest.
    try
    {
        _gen_recursive(p_type, p_dest_folder, processed);
    }
    catch (const std::exception &e)
    {
        RCLGD_FAIL_MSG(vformat("RCLGD: Failed to generate editor support for '%s': %s", p_type, e.what()));
    }
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
    String ros_interface_name = msg->get_ros_interface_name(); // e.g. "std_msgs/msg/String"

    // 1. Define the class and the metadata constant
    String code = "extends RosMsg\n";
    code += "class_name " + class_name + "\n\n";
    code += "const ROS_TYPE_NAME = \"" + ros_interface_name + "\"\n\n";

    // 2. Use the constant in the constructor
    code += "func _init():\n";
    code += "\tinit(ROS_TYPE_NAME)\n\n";

    CompoundMessage::SharedPtr babel = msg->get_babel();
    for (const std::string &key_str : babel->keys())
    {
        StringName field_sn(String(key_str.c_str()));
        String field = String(key_str.c_str());
        Variant value = msg->get_member(field_sn); // lazily convert this field
        String type_hint;

        if (value.get_type() == Variant::OBJECT)
        {
            Ref<RosMsg> nested = Object::cast_to<RosMsg>(value);
            type_hint = nested->get_type_name();
            _gen_recursive(nested->get_ros_interface_name(), p_dest_folder, p_processed);
        }
        else if (value.get_type() == Variant::ARRAY)
        {
            // An untyped Godot Array is either an array of compounds or of
            // bool/other primitives. (Packed arrays carry their own Variant type
            // and fall through to the default branch.) The cached Array may be
            // empty for a fresh message, so read the element type from BabelFish.
            std::string key = field.utf8().get_data();
            ros_babel_fish::Message &member = (*babel)[key];
            Ref<RosMsg> elem;
            if (member.type() == MessageTypes::Array &&
                member.as<ArrayMessageBase>().elementType() == MessageTypes::Compound)
            {
                RosTypeMapping::with_compound_array(member, [&](auto &arr) {
                    elem = RosMsg::from_type(String(arr.elementName().c_str()));
                });
            }

            if (elem.is_valid())
            {
                // Array[RosXxx]: element type matches the typed instances that
                // get_member hands back, so it satisfies the typed property.
                type_hint = "Array[" + elem->get_type_name() + "]";
                _gen_recursive(elem->get_ros_interface_name(), p_dest_folder, p_processed);
            }
            else
            {
                type_hint = "Array";
            }
        }
        else
        {
            type_hint = Variant::get_type_name(value.get_type());
        }

        // No cast needed: get_member returns the nested message with its shadow
        // script already attached, so it is genuinely a `type_hint`.
        code += "var " + field + " : " + type_hint + ":\n";
        code += "\tget: return get_member(&\"" + field + "\")\n";
        code += "\tset(v): set_member(&\"" + field + "\", v)\n\n";
    }

    DirAccess::make_dir_recursive_absolute(p_dest_folder);
    Ref<FileAccess> f = FileAccess::open(p_dest_folder.path_join(class_name + ".gd"), FileAccess::WRITE);
    if (f.is_valid())
        f->store_string(code);
}

void RosMsg::init_babel(const CompoundMessage::SharedPtr msg)
{
    ERR_FAIL_NULL_MSG(msg, "RCLGD: Cannot init_babel with a null shared_ptr.");
    msg_ = msg;
    members_.clear();

    // Members are NOT converted here — that happens lazily on first access in
    // get_member(). We only attach the typed shadow class (cheap, once) so this
    // object reports the right type; nested messages are wrapped (and scripted)
    // only when their field is actually read.
    _apply_script(this);
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
        if (member.type() == ros_babel_fish::MessageTypes::Compound)
        {
            // Wrap a nested compound. 'sub_ptr' aliases 'member' but shares the
            // ref-count with 'msg_', so the sub-message writes into our buffer.
            Ref<RosMsg> sub;
            sub.instantiate();
            auto sub_ptr = std::shared_ptr<ros_babel_fish::CompoundMessage>(
                msg_, &member.as<ros_babel_fish::CompoundMessage>());
            sub->init_babel(sub_ptr);
            value = sub;
        }
        else
        {
            // Leaf node: primitive or (packed/compound) array.
            ros_babel_fish::Message::SharedPtr member_ptr(msg_, &member);
            RBF2_TEMPLATE_CALL(Ros2Godot::call, member.type(), value, member_ptr);
        }
    }
    catch (const std::exception &e)
    {
        RCLGD_FAIL_V_MSG(Variant(), vformat("RCLGD: Failed to convert field '%s': %s", String(p_name), e.what()));
    }
    return value;
}

void RosMsg::init(const String &ros_type_name)
{
    // If msg_ is already set, we've already initialized this instance.
    // Re-entry is expected and benign: attaching the shadow script via set_script
    // re-runs the GDScript _init, which calls init() again on an already-built
    // message. Treat it as a deliberate no-op rather than warning.
    if (msg_)
    {
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
        RCLGD_FAIL_MSG(vformat("RCLGD: Failed to create ROS message '%s': %s", ros_type_name, e.what()));
        return;
    }
}

String RosMsg::shadow_class_name(const String &p_ros_datatype)
{
    // "std_msgs::msg::Header" -> "RosStdMsgsHeader"
    return "Ros" + p_ros_datatype.replace("::msg::", " ").replace("::", " ").to_pascal_case();
}

String RosMsg::get_type_name() const
{
    if (!msg_)
        return "RosUnknownType";
    // We use the C++ Datatype "std_msgs::msg::Header" to build the class name
    return shadow_class_name(String(msg_->datatype().c_str()));
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
    // If we have a script attached, let the script define the properties
    // to avoid double-entries in the inspector.
    if (!msg_ || get_script().get_type() != Variant::NIL)
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