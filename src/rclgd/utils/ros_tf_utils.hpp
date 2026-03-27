#ifndef ROS_TF_UTILS_HPP
#define ROS_TF_UTILS_HPP

#include <rclcpp/rclcpp.hpp>
#include <godot_cpp/variant/string.hpp>
#include <string>

namespace RclgdUtils {
    // Shared logic for resolving ~frame_id
    static std::string resolve_frame(std::shared_ptr<rclcpp::Node> n, const godot::String &p_id) {
        std::string s = p_id.utf8().get_data();
        if (s.empty()) return "";

        // Si empieza por ~, expandimos el namespace (p.ej. "/car" + "/" + "lidar")
        if (s[0] == '~' && n) 
            s = std::string(n->get_namespace()) + "/" + s.substr(1);

        // Eliminamos CUALQUIER barra inicial (TF2 no las tolera)
        size_t start = s.find_first_not_of('/');
        return (start == std::string::npos) ? "" : s.substr(start);
    }
}

#endif