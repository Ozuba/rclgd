// ros_msg.hpp
#ifndef ROS_MSG_TYPE_H
#define ROS_MSG_TYPE_H

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/variant.hpp>
#include <godot_cpp/variant/string_name.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/templates/hash_map.hpp> 
#include <godot_cpp/classes/dir_access.hpp>
#include <godot_cpp/classes/file_access.hpp>
#include <godot_cpp/classes/script.hpp>
#include <godot_cpp/templates/hash_set.hpp>

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

    private:
        //Fish dynamic support
        
        //The message its wrapping its compound because any message its a tree of object nodes with leafs ending up in a convertible ros to godot primitive
        CompoundMessage::SharedPtr msg_;

        //Stores Message contents
        HashMap<StringName, Variant> members_;


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

        //Internal recursive generator
        static void _gen_recursive(const String &p_type, const String &p_dest_folder, HashSet<String> &p_processed);

        //Initialization Method, makes the RosMsgType point to a Compound Message object
        void init(const String &ros_type_name);

        // Factory Builder method
        static Ref<RosMsg> from_type(const String &ros_type_name);

        // Generates editor autocomplete support in target folder
        static void gen_editor_support(const String &p_type, const String &p_dest_folder);

        //Alternate accesors to prevent recursion on inherited classes
        void set_member(const StringName &p_name, const Variant &p_value);
        Variant get_member(const StringName &p_name) const;

        // Return typename
        String get_type_name() const;
        String get_ros_interface_name() const ;

        // Map a ROS datatype ("geometry_msgs::msg::Pose") to its shadow class
        // name ("RosGeometryMsgsPose"). Shared by get_type_name and the array
        // element typing in ros_type_utils so both agree.
        static String shadow_class_name(const String &p_ros_datatype);

        // --- Typed (shadow) overlay ---
        // The dynamic C++ core is always correct on its own; the generated
        // `class_name RosXxx` scripts are an optional convenience layer on top.
        // To make typed access resolve to a real, typed instance, every message
        // built by init_babel gets the matching script attached (if generated).

        // Resolve the Script resource for a shadow class name (e.g. "RosStdMsgsHeader"),
        // using the project's global class list. Returns an invalid Ref if the type
        // was never generated, in which case the object stays a plain dynamic RosMsg.
        static Ref<Script> script_for_class(const String &p_class_name);

        // Attach the matching shadow script to a freshly built message (no-op if it
        // already carries a script, e.g. when created via `RosXxx.new()`).
        static void _apply_script(RosMsg *p_msg);

     
    };



    
#endif
