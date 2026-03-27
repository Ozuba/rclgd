#ifndef ROS_TF_UTILS_HPP
#define ROS_TF_UTILS_HPP

#include <rclcpp/rclcpp.hpp>
#include <godot_cpp/variant/string.hpp>
#include <string>

namespace RclgdUtils {
    // Shared logic for resolving ~frame_id
    static std::string resolve_frame(std::shared_ptr<rclcpp::Node> n, const godot::String &p_id) {
        std::string id_str = p_id.utf8().get_data();
        if (id_str.empty()) return "";

        if (id_str[0] == '~' && n) {
            std::string ns = n->get_namespace();
            std::string pure_id = id_str.substr(1);

            if (ns.empty() || ns == "/") return pure_id;
            return (ns.back() == '/') ? (ns + pure_id) : (ns + "/" + pure_id);
        }
        return id_str;
    }
}

#endif