#include "ros_type_registry.hpp"
#include "ros_msg.hpp"
#include "utils/rclgd_macros.hpp"
#include "utils/ros_type_utils.hpp"

#include <godot_cpp/classes/class_db_singleton.hpp>
#include <godot_cpp/classes/file_access.hpp>
#include <godot_cpp/classes/os.hpp>
#include <godot_cpp/classes/time.hpp>
#include <godot_cpp/classes/xml_parser.hpp>
#include <godot_cpp/core/method_ptrcall.hpp>
#include <godot_cpp/core/property_info.hpp>
#include <godot_cpp/templates/hash_set.hpp>
#include <godot_cpp/variant/typed_array.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

namespace
{
    // Owned TypeInfos keyed by BabelFish datatype. Their addresses (the
    // unique_ptrs) and their fields' (the vector is complete before any is
    // handed out) are stable, so they double as the class_userdata /
    // method_userdata given to the engine. Must be emptied at extension
    // de-init (they hold StringNames, which can't outlive the engine).
    std::unordered_map<std::string, std::unique_ptr<RosTypeRegistry::TypeInfo>> g_types;

    // Field names that can't become properties (see _is_reserved). Built on
    // first use during registration, cleared with the table.
    HashSet<StringName> g_reserved;

    // GDScript keywords (lowercase only: ROS field names are lowercase).
    const char *const GDSCRIPT_KEYWORDS[] = {
        "and", "as", "assert", "await", "break", "breakpoint", "class", "class_name",
        "const", "continue", "elif", "else", "enum", "extends", "false", "for", "func",
        "if", "in", "is", "match", "namespace", "not", "null", "or", "pass", "preload",
        "return", "self", "signal", "static", "super", "trait", "true", "var", "void",
        "when", "while", "yield",
    };

    // Class name of the typed instance currently being constructed on this
    // thread; read by RosMsgTypedCtor::_get_extension_class_name().
    thread_local const StringName *t_pending_class = nullptr;

    // Construction hook. godot-cpp's Wrapped ctor calls object_set_instance
    // exactly once, with the class name _pre_initialize<T>() took from
    // T::_get_extension_class_name(). RosMsg's (from GDCLASS) is fixed to
    // "RosMsg", so this layout-identical subclass shadows it to report the
    // typed class being built instead. That honours the documented contract
    // (one object_set_instance, most-derived class, during construction)
    // without a second set_instance overwriting the first.
    class RosMsgTypedCtor : public RosMsg
    {
    public:
        static const StringName *_get_extension_class_name() { return t_pending_class; }
    };
    static_assert(sizeof(RosMsgTypedCtor) == sizeof(RosMsg), "RosMsgTypedCtor must not add state: typed instances are freed as RosMsg");

    template <typename T>
    struct TypeTag
    {
        using type = T;
    };

    // Map a field's Variant type to the C++ type its ptrcall slot holds and
    // hand it to f. Covers every Variant type Ros2Godot produces; returns
    // false for anything else (such a field is left unregistered).
    template <typename F>
    bool with_ptr_type(Variant::Type p_type, F &&f)
    {
        switch (p_type)
        {
        case Variant::BOOL: f(TypeTag<bool>{}); return true;
        case Variant::INT: f(TypeTag<int64_t>{}); return true;
        case Variant::FLOAT: f(TypeTag<double>{}); return true;
        case Variant::STRING: f(TypeTag<String>{}); return true;
        case Variant::OBJECT: f(TypeTag<Ref<RosMsg>>{}); return true;
        case Variant::ARRAY: f(TypeTag<Array>{}); return true;
        case Variant::PACKED_BYTE_ARRAY: f(TypeTag<PackedByteArray>{}); return true;
        case Variant::PACKED_INT32_ARRAY: f(TypeTag<PackedInt32Array>{}); return true;
        case Variant::PACKED_INT64_ARRAY: f(TypeTag<PackedInt64Array>{}); return true;
        case Variant::PACKED_FLOAT32_ARRAY: f(TypeTag<PackedFloat32Array>{}); return true;
        case Variant::PACKED_FLOAT64_ARRAY: f(TypeTag<PackedFloat64Array>{}); return true;
        case Variant::PACKED_STRING_ARRAY: f(TypeTag<PackedStringArray>{}); return true;
        default: return false;
        }
    }

    bool is_supported_type(Variant::Type p_type)
    {
        return with_ptr_type(p_type, [](auto) {});
    }

    GDExtensionPropertyInfo c_property_info(const RosTypeRegistry::FieldInfo &p_field, const StringName &p_name)
    {
        return GDExtensionPropertyInfo{
            static_cast<GDExtensionVariantType>(p_field.type),
            p_name._native_ptr(),
            p_field.class_name._native_ptr(),
            p_field.hint,
            p_field.hint_string._native_ptr(),
            p_field.usage,
        };
    }

    inline RosMsg *self(GDExtensionClassInstancePtr p_instance)
    {
        return reinterpret_cast<RosMsg *>(p_instance);
    }

    inline const RosTypeRegistry::FieldInfo *field(void *p_method_userdata)
    {
        return static_cast<const RosTypeRegistry::FieldInfo *>(p_method_userdata);
    }
}

// --- Discovery ---

void RosTypeRegistry::register_project_types()
{
    // res:// and ProjectSettings are already set up at SCENE level: the
    // project is loaded before its extensions are initialized.
    const String manifest = "res://package.xml";
    if (!FileAccess::file_exists(manifest))
    {
        UtilityFunctions::print_verbose("rclgd: no res://package.xml, so no native message classes; every message is a dynamic RosMsg.");
        return;
    }

    const uint64_t start = Time::get_singleton()->get_ticks_usec();
    int declared = 0;
    for (const String &dep : _manifest_dependencies(manifest))
    {
        for (const String &type : _interfaces_of_package(dep))
        {
            ++declared;
            // Unresolvable types are reported by _create_babel and skipped.
            if (CompoundMessage::SharedPtr proto = RosMsg::_create_babel(type))
                _register(proto);
        }
    }
    g_reserved.clear(); // only needed while registering
    UtilityFunctions::print_verbose(vformat("rclgd: registered %d native message classes (%d declared interfaces + nested) in %.1f ms.",
                                            (int64_t)g_types.size(), declared, (Time::get_singleton()->get_ticks_usec() - start) / 1000.0));
}

PackedStringArray RosTypeRegistry::_manifest_dependencies(const String &p_manifest)
{
    PackedStringArray deps;
    Ref<XMLParser> parser;
    parser.instantiate();
    RCLGD_FAIL_COND_V_MSG(parser->open(p_manifest) != OK, deps, vformat("RCLGD: Could not parse %s.", p_manifest));

    String tag;
    while (parser->read() == OK)
    {
        switch (parser->get_node_type())
        {
        case XMLParser::NODE_ELEMENT:
            tag = parser->get_node_name();
            break;
        case XMLParser::NODE_TEXT:
            if (tag == "depend" || tag == "exec_depend" || tag == "build_depend")
            {
                String dep = parser->get_node_data().strip_edges();
                if (!dep.is_empty() && !deps.has(dep))
                    deps.push_back(dep);
            }
            break;
        case XMLParser::NODE_ELEMENT_END:
            tag = "";
            break;
        default:
            break;
        }
    }
    return deps;
}

PackedStringArray RosTypeRegistry::_interfaces_of_package(const String &p_package)
{
    // The rosidl ament index lists a package's interface files, e.g.
    // "msg/Twist.msg", "srv/AddTwoInts.idl", "action/Fibonacci.action".
    // Non-interface dependencies (rclgd itself, ...) have no marker.
    PackedStringArray out;
    for (const String &prefix : OS::get_singleton()->get_environment("AMENT_PREFIX_PATH").split(":", false))
    {
        const String marker = prefix.path_join("share/ament_index/resource_index/rosidl_interfaces").path_join(p_package);
        if (!FileAccess::file_exists(marker))
            continue;
        for (String line : FileAccess::get_file_as_string(marker).split("\n", false))
        {
            line = line.strip_edges();
            const String kind = line.get_slice("/", 0);
            const String base = vformat("%s/%s/%s", p_package, kind, line.get_slice("/", 1).get_basename());
            // A service is two messages and an action three (its internal
            // SendGoal/GetResult wrappers are not something scripts handle).
            std::vector<const char *> suffixes;
            if (kind == "msg")
                suffixes = {""};
            else if (kind == "srv")
                suffixes = {"_Request", "_Response"};
            else if (kind == "action")
                suffixes = {"_Goal", "_Result", "_Feedback"};
            for (const char *suffix : suffixes)
            {
                const String type = base + String(suffix);
                if (!out.has(type))
                    out.push_back(type);
            }
        }
        break; // first prefix providing the package wins (overlay order)
    }
    return out;
}

bool RosTypeRegistry::_is_reserved(const StringName &p_field)
{
    // A field can't be a property when its name is a GDScript keyword or
    // already means something on every message (a property, method, signal
    // or constant of RosMsg and its bases): the property would shadow it or
    // fail to parse. Such fields stay reachable through get_member/set_member.
    if (g_reserved.is_empty())
    {
        for (const char *keyword : GDSCRIPT_KEYWORDS)
            g_reserved.insert(StringName(keyword));
        g_reserved.insert(StringName("script")); // Object's, not in ClassDB

        ClassDBSingleton *cdb = ClassDBSingleton::get_singleton();
        const StringName base = RosMsg::get_class_static();
        TypedArray<Dictionary> methods = cdb->class_get_method_list(base, false);
        for (int i = 0; i < methods.size(); ++i)
            g_reserved.insert(Dictionary(methods[i])["name"]);
        TypedArray<Dictionary> properties = cdb->class_get_property_list(base, false);
        for (int i = 0; i < properties.size(); ++i)
            g_reserved.insert(Dictionary(properties[i])["name"]);
        TypedArray<Dictionary> signals = cdb->class_get_signal_list(base, false);
        for (int i = 0; i < signals.size(); ++i)
            g_reserved.insert(Dictionary(signals[i])["name"]);
        for (const String &constant : cdb->class_get_integer_constant_list(base, false))
            g_reserved.insert(StringName(constant));
    }
    return g_reserved.has(p_field);
}

// --- Registration ---

const RosTypeRegistry::TypeInfo *RosTypeRegistry::_register(const CompoundMessage::SharedPtr &p_proto)
{
    const std::string datatype = p_proto->datatype();
    auto existing = g_types.find(datatype);
    if (existing != g_types.end())
        return existing->second.get();

    auto info = std::make_unique<TypeInfo>();
    info->datatype = datatype;
    info->ros_name = String(p_proto->name().c_str());
    info->class_name = RosMsg::class_name_for_datatype(String(datatype.c_str()));

    // Work out each field's Godot type from the schema. Nested compound types
    // are registered first (depth-first), so every class a property names
    // already exists when this class is registered.
    try
    {
        for (const std::string &key : p_proto->keys())
        {
            ros_babel_fish::Message &member = (*p_proto)[key];
            FieldInfo f;
            f.name = StringName(String(key.c_str()));
            if (_is_reserved(f.name))
            {
                UtilityFunctions::print_verbose(vformat("rclgd: %s.%s clashes with a GDScript keyword or RosMsg member; reach it with get_member()/set_member().",
                                                        info->class_name, f.name));
                continue;
            }

            if (member.type() == MessageTypes::Compound)
            {
                // Alias into the prototype, same as _convert_member does.
                CompoundMessage::SharedPtr nested(p_proto, &member.as<CompoundMessage>());
                const TypeInfo *nested_type = _register(nested);
                if (!nested_type)
                    continue;
                f.type = Variant::OBJECT;
                f.class_name = nested_type->class_name;
                f.usage = PROPERTY_USAGE_NONE; // see below
            }
            else if (member.type() == MessageTypes::Array &&
                     member.as<ArrayMessageBase>().elementType() == MessageTypes::Compound)
            {
                // The prototype's array is empty, so build one element to recurse into.
                std::string elem_name;
                RosTypeMapping::with_compound_array(member, [&](auto &arr)
                                                    { elem_name = arr.elementName(); });
                CompoundMessage::SharedPtr elem = RosMsg::_create_babel(String(elem_name.c_str()));
                const TypeInfo *elem_type = elem ? _register(elem) : nullptr;
                if (!elem_type)
                    continue;
                f.type = Variant::ARRAY;
                f.hint = PROPERTY_HINT_ARRAY_TYPE;
                f.hint_string = String(elem_type->class_name);
                // Fields holding messages are neither stored nor editor-visible:
                // the engine caches a default value of every STORAGE/EDITOR
                // property (for docs and the inspector), and a cached message
                // object would outlive the unregistration of its class and be
                // freed through a dangling callback at engine shutdown (crash).
                // They stay typed, documented and autocompleted all the same.
                f.usage = PROPERTY_USAGE_NONE;
            }
            else
            {
                // Leaves: convert the default value once to learn the exact
                // Variant type Ros2Godot hands back (e.g. which Packed*Array).
                Variant probe;
                ros_babel_fish::Message::SharedPtr member_ptr(p_proto, &member);
                RBF2_TEMPLATE_CALL(Ros2Godot::call, member.type(), probe, member_ptr);
                f.type = probe.get_type();
            }

            if (!is_supported_type(f.type))
            {
                UtilityFunctions::print_verbose(vformat("rclgd: %s.%s has unsupported type %s; reach it with get_member().",
                                                        info->class_name, f.name, Variant::get_type_name(f.type)));
                continue;
            }
            f.getter = StringName("_ros_get_" + String(f.name));
            f.setter = StringName("_ros_set_" + String(f.name));
            info->fields.push_back(std::move(f));
        }
    }
    catch (const std::exception &e)
    {
        RCLGD_FAIL_V_MSG(nullptr, vformat("RCLGD: Cannot register native class for '%s': %s", info->ros_name, e.what()));
    }

    GDExtensionClassCreationInfo6 class_info = {};
    class_info.is_exposed = true; // GDScript only resolves exposed native classes
    // get_func/set_func stay null: field access then falls through to the
    // ClassDB properties registered below.
    class_info.to_string_func = &RosTypeRegistry::_to_string;
    class_info.create_instance_func = &RosTypeRegistry::_create_instance;
    class_info.free_instance_func = &RosTypeRegistry::_free_instance;
    class_info.class_userdata = info.get();

    ::godot::gdextension_interface::classdb_register_extension_class6(
        ::godot::gdextension_interface::library,
        info->class_name._native_ptr(),
        RosMsg::get_class_static()._native_ptr(),
        &class_info);

    for (const FieldInfo &f : info->fields)
        _register_field(*info, f);

    const TypeInfo *raw = info.get();
    g_types.emplace(datatype, std::move(info));
    return raw;
}

void RosTypeRegistry::_register_field(const TypeInfo &p_type, const FieldInfo &p_field)
{
    GDExtensionClassLibraryPtr library = ::godot::gdextension_interface::library;
    // The engine copies everything it needs out of these structs.
    StringName no_name;
    StringName value_name("value");

    GDExtensionPropertyInfo return_info = c_property_info(p_field, no_name);
    GDExtensionClassMethodInfo getter = {};
    getter.name = p_field.getter._native_ptr();
    getter.method_userdata = const_cast<FieldInfo *>(&p_field);
    getter.call_func = &RosTypeRegistry::_getter_call;
    getter.ptrcall_func = &RosTypeRegistry::_getter_ptrcall;
    getter.method_flags = GDEXTENSION_METHOD_FLAG_NORMAL | GDEXTENSION_METHOD_FLAG_CONST;
    getter.has_return_value = true;
    getter.return_value_info = &return_info;
    getter.return_value_metadata = GDEXTENSION_METHOD_ARGUMENT_METADATA_NONE;
    ::godot::gdextension_interface::classdb_register_extension_class_method(library, p_type.class_name._native_ptr(), &getter);

    GDExtensionPropertyInfo arg_info = c_property_info(p_field, value_name);
    GDExtensionClassMethodArgumentMetadata arg_meta = GDEXTENSION_METHOD_ARGUMENT_METADATA_NONE;
    GDExtensionClassMethodInfo setter = {};
    setter.name = p_field.setter._native_ptr();
    setter.method_userdata = const_cast<FieldInfo *>(&p_field);
    setter.call_func = &RosTypeRegistry::_setter_call;
    setter.ptrcall_func = &RosTypeRegistry::_setter_ptrcall;
    setter.method_flags = GDEXTENSION_METHOD_FLAG_NORMAL;
    setter.argument_count = 1;
    setter.arguments_info = &arg_info;
    setter.arguments_metadata = &arg_meta;
    ::godot::gdextension_interface::classdb_register_extension_class_method(library, p_type.class_name._native_ptr(), &setter);

    GDExtensionPropertyInfo prop_info = c_property_info(p_field, p_field.name);
    ::godot::gdextension_interface::classdb_register_extension_class_property(
        library, p_type.class_name._native_ptr(), &prop_info, p_field.setter._native_ptr(), p_field.getter._native_ptr());
}

void RosTypeRegistry::unregister_all()
{
    // Order doesn't matter: typed classes all derive from RosMsg directly,
    // never from each other.
    for (const auto &entry : g_types)
        ::godot::gdextension_interface::classdb_unregister_extension_class(
            ::godot::gdextension_interface::library, entry.second->class_name._native_ptr());
    g_types.clear();
    g_reserved.clear();
}

void RosTypeRegistry::load_editor_docs()
{
    // One class reference per type, built from the registered fields, so the
    // typed classes show up documented in the editor help and search. Handed
    // over in a single buffer of concatenated documents, as godot-cpp does
    // with its own doc data.
    String xml;
    for (const auto &entry : g_types)
    {
        const TypeInfo *type = entry.second.get();
        xml += "<?xml version=\"1.0\" encoding=\"UTF-8\" ?>\n";
        xml += vformat("<class name=\"%s\" inherits=\"RosMsg\" api_type=\"extension\">\n", type->class_name);
        xml += vformat("\t<brief_description>\n\t\tROS 2 interface [code]%s[/code].\n\t</brief_description>\n", type->ros_name);
        xml += vformat("\t<description>\n\t\tNative class for [code]%s[/code], registered by rclgd at startup from the project's [code]package.xml[/code]. "
                       "Each field is a typed property; fields whose names clash with [RosMsg] members or GDScript keywords are only reachable through [method RosMsg.get_member] and [method RosMsg.set_member]. "
                       "Instances are created with [code]new()[/code] or [method RosMsg.from_type], and received messages of this type arrive as this class.\n\t</description>\n",
                       type->ros_name);
        xml += "\t<tutorials>\n\t</tutorials>\n\t<members>\n";
        for (const FieldInfo &f : type->fields)
        {
            String member_type;
            if (f.type == Variant::OBJECT)
                member_type = f.class_name;
            else if (f.hint == PROPERTY_HINT_ARRAY_TYPE)
                member_type = f.hint_string + String("[]");
            else
                member_type = Variant::get_type_name(f.type);
            xml += vformat("\t\t<member name=\"%s\" type=\"%s\" setter=\"%s\" getter=\"%s\">\n\t\t\tField [code]%s[/code] of [code]%s[/code].\n\t\t</member>\n",
                           f.name, member_type, f.setter, f.getter, f.name, type->ros_name);
        }
        xml += "\t</members>\n</class>\n";
    }
    if (!xml.is_empty())
        ::godot::gdextension_interface::editor_help_load_xml_from_utf8_chars(xml.utf8().get_data());
}

const RosTypeRegistry::TypeInfo *RosTypeRegistry::find(const std::string &p_datatype)
{
    auto it = g_types.find(p_datatype);
    return it != g_types.end() ? it->second.get() : nullptr;
}

// --- Instances ---

RosMsg *RosTypeRegistry::_construct(const TypeInfo *p_type)
{
    // First half of an expanded memnew: the engine object comes back holding
    // one reference that the caller must either adopt into a Ref (C++ path)
    // or hand back to the engine (create_instance path).
    t_pending_class = &p_type->class_name;
    ::godot::_pre_initialize<RosMsgTypedCtor>();
    t_pending_class = nullptr;
    RosMsg *msg = new ("", "") RosMsgTypedCtor;
    msg->typed_ = true;
    return msg;
}

Ref<RosMsg> RosTypeRegistry::instantiate(const TypeInfo *p_type, const CompoundMessage::SharedPtr &p_msg)
{
    // Second half of memnew. On Godot >= 4.7 _post_initialize returns a Ref
    // that adopts the initial reference; discarding it would free the object
    // on the spot, and Ref<RosMsg>(raw_ptr) would add a second one (leak).
    Ref<RosMsg> ref = ::godot::_post_initialize(static_cast<RosMsgTypedCtor *>(_construct(p_type)));
    ref->init_babel(p_msg);
    return ref;
}

GDExtensionObjectPtr RosTypeRegistry::_create_instance(void *p_class_userdata, GDExtensionBool p_notify_postinitialize)
{
    // `RosXxx.new()`: build a fresh message of this type. init() reports and
    // swallows BabelFish failures, leaving an empty (but valid) object.
    const TypeInfo *type = static_cast<const TypeInfo *>(p_class_userdata);
    RosMsg *msg = _construct(type);
    // Same as godot-cpp's own create_instance: notify without adopting the
    // reference, which belongs to the engine-side caller.
    if (p_notify_postinitialize)
        postinitialize_handler(static_cast<Wrapped *>(msg));
    msg->init(type->ros_name);
    return msg->_owner;
}

void RosTypeRegistry::_free_instance(void * /*p_class_userdata*/, GDExtensionClassInstancePtr p_instance)
{
    if (!p_instance)
        return;
    // Mirrors how memnew allocated it (operator new -> Memory::alloc_static).
    RosMsg *msg = self(p_instance);
    msg->~RosMsg();
    Memory::free_static(msg);
}

void RosTypeRegistry::_to_string(GDExtensionClassInstancePtr p_instance, GDExtensionBool *r_is_valid, GDExtensionStringPtr r_out)
{
    if (!p_instance)
        return;
    *reinterpret_cast<String *>(r_out) = self(p_instance)->_to_string();
    *r_is_valid = true;
}

// --- Field accessors ---

void RosTypeRegistry::_getter_call(void *p_method_userdata, GDExtensionClassInstancePtr p_instance, const GDExtensionConstVariantPtr * /*p_args*/, GDExtensionInt p_argument_count, GDExtensionVariantPtr r_return, GDExtensionCallError *r_error)
{
    if (p_argument_count != 0)
    {
        r_error->error = GDEXTENSION_CALL_ERROR_TOO_MANY_ARGUMENTS;
        r_error->expected = 0;
        return;
    }
    r_error->error = GDEXTENSION_CALL_OK;
    Variant value = self(p_instance)->get_member(field(p_method_userdata)->name);
    // r_return is an uninitialized Variant slot (see godot-cpp MethodBind::bind_call).
    ::godot::gdextension_interface::variant_new_copy(r_return, value._native_ptr());
}

void RosTypeRegistry::_getter_ptrcall(void *p_method_userdata, GDExtensionClassInstancePtr p_instance, const GDExtensionConstTypePtr * /*p_args*/, GDExtensionTypePtr r_ret)
{
    const FieldInfo *f = field(p_method_userdata);
    Variant value = self(p_instance)->get_member(f->name);
    with_ptr_type(f->type, [&](auto tag)
                  {
        using T = typename decltype(tag)::type;
        PtrToArg<T>::encode(static_cast<T>(value), r_ret); });
}

void RosTypeRegistry::_setter_call(void *p_method_userdata, GDExtensionClassInstancePtr p_instance, const GDExtensionConstVariantPtr *p_args, GDExtensionInt p_argument_count, GDExtensionVariantPtr /*r_return*/, GDExtensionCallError *r_error)
{
    if (p_argument_count != 1)
    {
        r_error->error = p_argument_count < 1 ? GDEXTENSION_CALL_ERROR_TOO_FEW_ARGUMENTS : GDEXTENSION_CALL_ERROR_TOO_MANY_ARGUMENTS;
        r_error->expected = 1;
        return;
    }
    r_error->error = GDEXTENSION_CALL_OK;
    self(p_instance)->set_member(field(p_method_userdata)->name, *reinterpret_cast<const Variant *>(p_args[0]));
}

void RosTypeRegistry::_setter_ptrcall(void *p_method_userdata, GDExtensionClassInstancePtr p_instance, const GDExtensionConstTypePtr *p_args, GDExtensionTypePtr /*r_ret*/)
{
    const FieldInfo *f = field(p_method_userdata);
    Variant value;
    with_ptr_type(f->type, [&](auto tag)
                  {
        using T = typename decltype(tag)::type;
        value = Variant(PtrToArg<T>::convert(p_args[0])); });
    self(p_instance)->set_member(f->name, value);
}
