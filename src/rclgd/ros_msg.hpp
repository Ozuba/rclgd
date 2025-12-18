// ros_node.h
#ifndef ROS_MSG_TYPE_H
#define ROS_MSG_TYPE_H

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/variant.hpp>
#include <godot_cpp/variant/string_name.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/templates/hash_map.hpp> // Critical include

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

        // Internal initialization, makes the RosMsgType point to a Compound Message object
        void init(CompoundMessage::SharedPtr p_msg);

        //Factory Builder method
        static Ref<RosMsg> from_type(const String &ros_typename);

        //Internal Fish Message Instance Getter
        CompoundMessage::SharedPtr get_msg() const { return msg_; }
     
    };



    
#endif
