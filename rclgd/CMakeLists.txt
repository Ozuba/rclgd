cmake_minimum_required(VERSION 3.11)
project(rclgd)

# Compiler options
add_compile_options("-O3")
find_package(ament_cmake REQUIRED)
#####################################################[ GODOT RUNTIME ]##########################################################
# The engine binary is a *runtime* dependency: librclgd.so only needs godot-cpp
# to compile. The build just records the required version and `ros2 rclgd
# setup` downloads the editor binary into lib/rclgd/ afterwards, keeping the
# build network-free (buildfarm/offline friendly).

# The one supported engine version: the version the godot-cpp submodule (and
# therefore librclgd.so) is built against. `ros2 rclgd setup` downloads
# exactly this release. To target another version, retarget the submodule and
# edit this line — a plain variable (not CACHE), so the edit always takes
# effect on rebuild.
set(GODOT_VERSION "4.7.1")
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/godot-cpp/gdextension/extension_api.json" _gdcpp_api_head LIMIT 2048)
string(REGEX MATCH "\"version_major\"[ \t]*:[ \t]*([0-9]+)" _unused "${_gdcpp_api_head}")
set(_gdcpp_api_major "${CMAKE_MATCH_1}")
string(REGEX MATCH "\"version_minor\"[ \t]*:[ \t]*([0-9]+)" _unused "${_gdcpp_api_head}")
set(_gdcpp_api_minor "${CMAKE_MATCH_1}")
if(_gdcpp_api_major AND _gdcpp_api_minor
        AND NOT GODOT_VERSION MATCHES "^${_gdcpp_api_major}\\.${_gdcpp_api_minor}(\\.|$)")
    message(WARNING
        "rclgd: GODOT_VERSION (${GODOT_VERSION}) does not match the godot-cpp submodule "
        "(API ${_gdcpp_api_major}.${_gdcpp_api_minor}). Retarget the submodule to the "
        "matching branch so the downloaded engine and the bindings agree.")
endif()
# Version pin consumed by `ros2 rclgd setup` (one line, e.g. "4.7.1")
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/godot_version" "${GODOT_VERSION}\n")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/godot_version" DESTINATION share/${PROJECT_NAME})

# Launcher: exec's the engine binary that `ros2 rclgd setup` downloads as
# lib/rclgd/godot-bin (next to librclgd.so, where GDExtension resolves the
# extension library). Installed as lib/rclgd/godot so `ros2 run rclgd godot`
# works.
install(PROGRAMS cmake/godot-launcher.sh DESTINATION lib/${PROJECT_NAME} RENAME godot)
#####################################################[ (rclgd ) ]########################################################

find_package(ros_babel_fish REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Build godot-cpp
# Need shaderd libstdc to catch external exception errors from babel_fish or other dloaded libs
set(GODOTCPP_USE_STATIC_CPP OFF CACHE BOOL "Link libgcc/libstdc++ statically" FORCE)
add_subdirectory(godot-cpp)
# Add godot-cpp cmake module for documentation embedding
list(APPEND CMAKE_MODULE_PATH "${godot-cpp_SOURCE_DIR}/cmake")
include(GodotCPPModule)

set(GODOTCPP_DISABLE_EXCEPTIONS OFF)
add_library(godot_cpp_no_cmake_warnings INTERFACE)
target_link_libraries(godot_cpp_no_cmake_warnings INTERFACE godot::cpp)
target_compile_options(godot_cpp_no_cmake_warnings INTERFACE -w)


# Extension Source
include_directories(src ${rclcpp_INCLUDE_DIRS} ${ros_babel_fish_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS})

# Fetch a list of the xml files to use for documentation and add to our target
file(GLOB_RECURSE DOC_XML LIST_DIRECTORIES NO CONFIGURE_DEPENDS "doc_classes/*.xml")

add_library(rclgd SHARED)

target_sources(rclgd PRIVATE
    # RCLGD
    src/rclgd/rclgd.cpp
    src/rclgd/ros_msg.cpp
    src/rclgd/ros_node.cpp
    src/rclgd/ros_publisher.cpp
    src/rclgd/ros_subscriber.cpp
    src/rclgd/ros_client.cpp
    src/rclgd/ros_service.cpp
    src/rclgd/ros_action_client.cpp
    src/rclgd/ros_action_server.cpp
    src/rclgd/ros_tf_broadcaster.cpp
    src/rclgd/ros_tf_listener.cpp
    src/rclgd/ros_qos.cpp
    src/rclgd/ros_timer.cpp

    # Type Registration
    src/register_types.cpp

)
# Add docs sources
target_doc_sources(rclgd "${DOC_XML}")

ament_target_dependencies(rclgd rclcpp ros_babel_fish tf2_ros)
target_link_libraries(rclgd godot_cpp_no_cmake_warnings)


if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_link_options(rclgd PRIVATE -Wl,--exclude-libs,ALL)
endif()



#####################################################[ INSTALLATION ]###############################################################

install(TARGETS rclgd DESTINATION lib/${PROJECT_NAME})
# Expose the shared library for godot packages to pick
ament_export_libraries(rclgd)
ament_package()