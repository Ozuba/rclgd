// ros_type_registry.hpp
#ifndef ROS_TYPE_REGISTRY_H
#define ROS_TYPE_REGISTRY_H

#include <godot_cpp/classes/ref.hpp>
#include <godot_cpp/core/property_info.hpp>
#include <godot_cpp/variant/packed_string_array.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/string_name.hpp>
#include <godot_cpp/variant/variant.hpp>

#include <ros_babel_fish/messages/compound_message.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

using namespace godot;

class RosMsg;

// Native, typed ClassDB classes for ROS message types.
//
// Each registered ROS type (e.g. geometry_msgs/msg/Twist) becomes a real
// engine class (RosGeometryMsgsTwist) deriving from RosMsg, with one typed
// property per field. Nothing about the dynamic core changes: a typed class is
// just a RosMsg whose engine-side class is more derived, and whose properties
// are backed by RosMsg::get_member/set_member. Unregistered types keep working
// as plain dynamic RosMsg.
//
// The table is filled once on the main thread during extension init and only
// read afterwards (also from the ROS executor thread, via RosMsg::wrap), so
// lookups need no lock. It is torn down in extension de-init, after the
// executor has been joined.
class RosTypeRegistry
{
public:
    struct FieldInfo
    {
        StringName name;
        StringName getter;
        StringName setter;
        Variant::Type type = Variant::NIL;
        StringName class_name; // nested message class, or element class of Array[RosXxx]
        uint32_t hint = 0;
        String hint_string;
        uint32_t usage = PROPERTY_USAGE_DEFAULT;
    };

    struct TypeInfo
    {
        std::string datatype; // "geometry_msgs::msg::Twist" (BabelFish datatype(), the lookup key)
        String ros_name;      // "geometry_msgs/msg/Twist"   (for create_message_shared)
        StringName class_name; // "RosGeometryMsgsTwist"
        std::vector<FieldInfo> fields;
    };

    // Register every interface type the project depends on: the dependencies
    // of res://package.xml, resolved through the ament index (msg, srv
    // Request/Response, action Goal/Result/Feedback), plus everything they
    // nest. Extension init (SCENE level) only; the set is fixed from then on.
    static void register_project_types();

    // Hand the editor help a class reference for every registered type.
    // EDITOR initialization level only (the help system exists from there).
    static void load_editor_docs();

    // Unregister every class and drop the table. MUST run during extension
    // de-init, after the executor is joined and before godot-cpp unregisters
    // RosMsg (the parent of all typed classes).
    static void unregister_all();

    // Typed class for a BabelFish datatype, or nullptr if not registered.
    static const TypeInfo *find(const std::string &p_datatype);

    // Build a typed instance of p_type wrapping p_msg (no resource loading;
    // safe on the executor thread).
    static Ref<RosMsg> instantiate(const TypeInfo *p_type, const ros_babel_fish::CompoundMessage::SharedPtr &p_msg);

private:
    static PackedStringArray _manifest_dependencies(const String &p_manifest);
    static PackedStringArray _interfaces_of_package(const String &p_package);
    static bool _is_reserved(const StringName &p_field);
    static const TypeInfo *_register(const ros_babel_fish::CompoundMessage::SharedPtr &p_proto);
    static void _register_field(const TypeInfo &p_type, const FieldInfo &p_field);
    static RosMsg *_construct(const TypeInfo *p_type);

    // Engine callbacks, written against the GDExtension C interface. The engine
    // only invokes free/to_string of the most-derived class, so RosMsg's
    // (GDCLASS-generated) ones never run for typed instances. Neither class
    // handles notifications, so that slot stays empty.
    static GDExtensionObjectPtr _create_instance(void *p_class_userdata, GDExtensionBool p_notify_postinitialize);
    static void _free_instance(void *p_class_userdata, GDExtensionClassInstancePtr p_instance);
    static void _to_string(GDExtensionClassInstancePtr p_instance, GDExtensionBool *r_is_valid, GDExtensionStringPtr r_out);

    // One typed getter/setter pair per field; method_userdata is the FieldInfo.
    // Typed per field on purpose: GDScript takes a native property's static
    // type from its getter's return type, so a shared Variant accessor would
    // leave every field untyped.
    static void _getter_call(void *p_method_userdata, GDExtensionClassInstancePtr p_instance, const GDExtensionConstVariantPtr *p_args, GDExtensionInt p_argument_count, GDExtensionVariantPtr r_return, GDExtensionCallError *r_error);
    static void _getter_ptrcall(void *p_method_userdata, GDExtensionClassInstancePtr p_instance, const GDExtensionConstTypePtr *p_args, GDExtensionTypePtr r_ret);
    static void _setter_call(void *p_method_userdata, GDExtensionClassInstancePtr p_instance, const GDExtensionConstVariantPtr *p_args, GDExtensionInt p_argument_count, GDExtensionVariantPtr r_return, GDExtensionCallError *r_error);
    static void _setter_ptrcall(void *p_method_userdata, GDExtensionClassInstancePtr p_instance, const GDExtensionConstTypePtr *p_args, GDExtensionTypePtr r_ret);
};

#endif
