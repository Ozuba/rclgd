#ifndef ROS_TYPE_UTILS_HPP
#define ROS_TYPE_UTILS_HPP

#include <godot_cpp/variant/variant.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/packed_byte_array.hpp>
#include <godot_cpp/variant/packed_float32_array.hpp>
#include <godot_cpp/variant/packed_float64_array.hpp>
#include <godot_cpp/variant/packed_int32_array.hpp>
#include <godot_cpp/variant/packed_int64_array.hpp>
#include <godot_cpp/variant/packed_string_array.hpp>
#include <godot_cpp/variant/array.hpp>

#include <ros_babel_fish/babel_fish.hpp>
#include <ros_babel_fish/macros.hpp>
#include <ros_babel_fish/messages/message.hpp>
#include <ros_babel_fish/messages/compound_message.hpp>
#include <ros_babel_fish/messages/array_message.hpp>
#include <ros_babel_fish/messages/value_message.hpp>

#include "../ros_msg.hpp"

#include <type_traits>
#include <string>
#include <algorithm>
#include <cstring>

using namespace godot;
using namespace ros_babel_fish;

namespace RosTypeMapping {

    // Helper to safely get the address of the underlying data in a BabelFish array.
    // BabelFish storage for arithmetic types is always contiguous.
    template<typename T, bool B, bool FL>
    static void* get_ros_ptr(ArrayMessage_<T, B, FL> &ros_array) {
        if (ros_array.size() == 0) return nullptr;
        return (void*)(&ros_array[0]);
    }

    // --- ROS TO GODOT (Reading/Subscribing) ---
    struct RosToGodotProvider {
        template <typename T, bool B, bool FL>
        static void handle(ArrayMessage_<T, B, FL> &ros_array, Variant &r_ret) {
            size_t size = ros_array.size();

            // FAST PATH: uint8_t -> PackedByteArray
            if constexpr (std::is_same_v<T, uint8_t>) {
                PackedByteArray packed; packed.resize(size);
                if (size > 0) std::memcpy(packed.ptrw(), get_ros_ptr(ros_array), size);
                r_ret = packed; return;
            }
            // FAST PATH: float -> PackedFloat32Array
            else if constexpr (std::is_same_v<T, float>) {
                PackedFloat32Array packed; packed.resize(size);
                if (size > 0) std::memcpy(packed.ptrw(), get_ros_ptr(ros_array), size * sizeof(float));
                r_ret = packed; return;
            }
            // FAST PATH: double -> PackedFloat64Array
            else if constexpr (std::is_same_v<T, double>) {
                PackedFloat64Array packed; packed.resize(size);
                if (size > 0) std::memcpy(packed.ptrw(), get_ros_ptr(ros_array), size * sizeof(double));
                r_ret = packed; return;
            }

            // SLOW PATH: Strings or other generic types
            if constexpr (std::is_same_v<T, std::string>) {
                PackedStringArray packed; packed.resize(size);
                for (size_t i = 0; i < size; ++i) packed.set(i, ros_array[i].c_str());
                r_ret = packed;
            } else {
                Array arr; arr.resize(size);
                for (size_t i = 0; i < size; ++i) {
                    if constexpr (std::is_arithmetic_v<T>) arr[i] = static_cast<double>(ros_array[i]);
                    else arr[i] = Variant();
                }
                r_ret = arr;
            }
        }
        
        template <bool B, bool FL>
        static void handle(CompoundArrayMessage_<B, FL> &ros_array, Variant &r_ret) {}
    };

    // --- GODOT TO ROS (Writing/Publishing) ---
    struct GodotToRosProvider {
        template <typename T, bool B, bool FL>
        static void handle(ArrayMessage_<T, B, FL> &ros_array, const Variant &p_val) {
            
            // FAST PATH: PackedByteArray -> uint8_t
            if constexpr (std::is_same_v<T, uint8_t>) {
                if (p_val.get_type() == Variant::PACKED_BYTE_ARRAY) {
                    PackedByteArray pba = p_val;
                    if constexpr (!FL) ros_array.resize(pba.size());
                    size_t limit = std::min((size_t)pba.size(), (size_t)ros_array.size());
                    if (limit > 0) std::memcpy(get_ros_ptr(ros_array), pba.ptr(), limit);
                    return;
                }
            }
            // FAST PATH: PackedFloat32Array -> float
            else if constexpr (std::is_same_v<T, float>) {
                if (p_val.get_type() == Variant::PACKED_FLOAT32_ARRAY) {
                    PackedFloat32Array pfa = p_val;
                    if constexpr (!FL) ros_array.resize(pfa.size());
                    size_t limit = std::min((size_t)pfa.size(), (size_t)ros_array.size());
                    if (limit > 0) std::memcpy(get_ros_ptr(ros_array), pfa.ptr(), limit * sizeof(float));
                    return;
                }
            }

            // SLOW PATH: Strings, Bools, or Mixed Arrays
            Array arr_view = p_val;
            size_t g_size = arr_view.size();
            if constexpr (!FL) ros_array.resize(g_size);
            size_t limit = std::min((size_t)ros_array.size(), g_size);
            for (size_t i = 0; i < limit; ++i) {
                Variant item = arr_view[i];
                if constexpr (std::is_same_v<T, std::string>) ros_array[i] = item.operator String().utf8().get_data();
                else if constexpr (std::is_arithmetic_v<T>) ros_array[i] = static_cast<T>(item.operator double());
            }
        }

        template <bool B, bool FL>
        static void handle(CompoundArrayMessage_<B, FL> &ros_array, const Variant &p_val) {}
    };
}

// --- PRIMARY DISPATCHERS ---

struct Ros2Godot {
    template <typename T>
    static void call(Variant &r_ret, Message::SharedPtr p_msg) {
        //Handle Compound
        if constexpr (std::is_same_v<T, CompoundMessage>) {
            Ref<RosMsg> sub; sub.instantiate();
            sub->init_babel(std::static_pointer_cast<CompoundMessage>(p_msg));
            r_ret = sub;
        } 
        //Handle Array
        else if constexpr (std::is_base_of_v<ArrayMessageBase, T>) {
            auto &base_arr = p_msg->as<ArrayMessageBase>();
            if (base_arr.elementType() == MessageTypes::Compound) {
                auto &ros_array = p_msg->as<CompoundArrayMessage>();
                Array g_arr; g_arr.resize(ros_array.size());
                for (size_t i = 0; i < ros_array.size(); ++i) {
                    Variant el;
                    RBF2_TEMPLATE_CALL(Ros2Godot::call, MessageTypes::Compound, el, ros_array.values()[i]);
                    g_arr[i] = el;
                }
                r_ret = g_arr;
            } else {
                RBF2_TEMPLATE_CALL_ARRAY_TYPES(RosTypeMapping::RosToGodotProvider::handle, base_arr, r_ret);
            }
        }
        //Handle Primitive
        else {
            auto &val_msg = p_msg->as<ValueMessage<T>>();
            if constexpr (std::is_same_v<T, std::string>) r_ret = String(val_msg.getValue().c_str());
            else if constexpr (std::is_integral_v<T>) r_ret = (int64_t)val_msg.getValue();
            else if constexpr (std::is_floating_point_v<T>) r_ret = (double)val_msg.getValue();
            else if constexpr (std::is_same_v<T, bool>) r_ret = val_msg.getValue();
            else r_ret = Variant();
        }
    }
};

struct Godot2Ros {
    template <typename T>
    static void call(const Variant &p_val, Message &p_ros_msg) {
        if constexpr (std::is_same_v<T, CompoundMessage>) {
            Ref<RosMsg> sub = p_val;
            if (sub.is_valid() && sub->get_babel()) p_ros_msg.as<CompoundMessage>() = *(sub->get_babel());
        } else if constexpr (std::is_base_of_v<ArrayMessageBase, T>) {
            auto &base_arr = p_ros_msg.as<ArrayMessageBase>();
            if (base_arr.elementType() == MessageTypes::Compound) {
                auto &ros_array = p_ros_msg.as<CompoundArrayMessage>();
                Array g_arr = p_val;
                if (!ros_array.isFixedSize()) ros_array.resize(g_arr.size());
                size_t limit = std::min((size_t)ros_array.size(), (size_t)g_arr.size());
                for (size_t i = 0; i < limit; ++i) {
                    Ref<RosMsg> sub = g_arr[i];
                    if (sub.is_valid() && sub->get_babel()) ros_array[i] = *(sub->get_babel());
                }
            } else {
                RBF2_TEMPLATE_CALL_ARRAY_TYPES(RosTypeMapping::GodotToRosProvider::handle, base_arr, p_val);
            }
        } else {
            auto &val_msg = p_ros_msg.as<ValueMessage<T>>();
            if constexpr (std::is_same_v<T, std::string>) val_msg.setValue(p_val.operator String().utf8().get_data());
            else if constexpr (std::is_integral_v<T>) val_msg.setValue((T)p_val.operator int64_t());
            else if constexpr (std::is_floating_point_v<T>) val_msg.setValue((T)p_val.operator double());
            else if constexpr (std::is_same_v<T, bool>) val_msg.setValue(p_val.operator bool());
        }
    }
};

#endif