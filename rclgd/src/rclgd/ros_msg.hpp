// ros_msg.hpp
#ifndef ROS_MSG_TYPE_H
#define ROS_MSG_TYPE_H

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/variant.hpp>
#include <godot_cpp/variant/string_name.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/templates/hash_map.hpp> 

//BabelFish
#include <ros_babel_fish/babel_fish.hpp>
#include <ros_babel_fish/messages/array_message.hpp>
#include <ros_babel_fish/messages/compound_message.hpp>
#include <ros_babel_fish/messages/value_message.hpp>

#include <rclcpp/rclcpp.hpp>
#include "rclgd.hpp"


using namespace ros_babel_fish; 
using namespace godot;

    //Esentially behaves as a compound message
    class RosMsg : public RefCounted
    {
        GDCLASS(RosMsg, RefCounted)
        friend class RosTypeRegistry;

    private:
        //Fish dynamic support

        //The message its wrapping its compound because any message its a tree of object nodes with leafs ending up in a convertible ros to godot primitive
        CompoundMessage::SharedPtr msg_;

        // Lazy Godot-side cache of converted members. Members are converted from
        // the BabelFish buffer on first access (see get_member) rather than all
        // up front, so receiving a large message you only read a field or two of
        // stays cheap. Mutable because population happens inside const accessors.
        mutable HashMap<StringName, Variant> members_;

        // True when this instance is a native typed class (RosGeometryMsgsTwist,
        // ...) built by RosTypeRegistry. Its fields are then real ClassDB
        // properties, so the dynamic property list is skipped.
        bool typed_ = false;

        // Convert a single member (compound -> nested RosMsg, leaf -> Variant)
        // straight from the BabelFish buffer, without consulting/updating the cache.
        Variant _convert_member(const StringName &p_name) const;

        // Allocate a BabelFish message of the given type; reports and returns
        // null if the type can't be resolved. Shared by init() and from_type().
        static CompoundMessage::SharedPtr _create_babel(const String &p_ros_type);


    protected:
        static void _bind_methods();
        void _get_property_list(List<PropertyInfo> *r_props)const;      // return list of properties
        bool _get(const StringName &p_property, Variant &r_value) const; // return true if property was found
        bool _set(const StringName &p_property, const Variant &p_value); // return true if property was found
        String _to_string() const; //String Representation
    public:
        RosMsg();
        ~RosMsg();

        //Internal initializer
        void init_babel(const CompoundMessage::SharedPtr msg);

        // Internal Fish Message Instance Getter nad getter
        CompoundMessage::SharedPtr get_babel() const { return msg_; }

        //GODOT EXPOSED

        //Initialization Method, makes the RosMsgType point to a Compound Message object
        void init(const String &ros_type_name);

        // Factory Builder method
        static Ref<RosMsg> from_type(const String &ros_type_name);

        // Central factory for every message wrapper built from C++ (received,
        // nested, service/action payloads). Returns the native typed class when
        // the ROS type is registered, a dynamic RosMsg otherwise, and an empty
        // RosMsg for a null message (e.g. a missing action result). Only reads
        // the registry table, so it is safe on the executor thread.
        static Ref<RosMsg> wrap(const CompoundMessage::SharedPtr &p_msg);

        // Field access by name: the single entry point to the lazy cache. The
        // native typed classes' property accessors delegate here, and it is how
        // scripts reach fields that are not exposed as properties (name clashes,
        // unsupported types) or are only known at runtime.
        void set_member(const StringName &p_name, const Variant &p_value);
        Variant get_member(const StringName &p_name) const;

        // Return typename
        String get_type_name() const;
        String get_ros_interface_name() const ;

        // Map a ROS datatype ("geometry_msgs::msg::Pose") to its class name
        // ("RosGeometryMsgsPose"). Shared by get_type_name and the native class
        // registry so both agree.
        static String class_name_for_datatype(const String &p_ros_datatype);

    };



    
#endif
