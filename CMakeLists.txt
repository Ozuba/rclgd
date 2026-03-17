cmake_minimum_required(VERSION 3.11)
project(rclgd)

# Compiler options
add_compile_options("-O3")
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
include(FetchContent)
#####################################################[ DEPENDENCIES ]###########################################################

# 1. Determine architecture-specific strings for the URL
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    set(ARCH_SUFFIX "x86_64")
    set(GODOT_PLATFORM "linux.64")
    set(GODOT_SLUG "linux.x86_64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(ARCH_SUFFIX "arm64")
    set(GODOT_PLATFORM "linux.arm64")
    set(GODOT_SLUG "linux.arm64")
else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}.")
endif()

set(GODOT_VERSION "4.6")

# 2. Build the URL using the architecture-aware variables
set(GODOT_URL "https://downloads.godotengine.org/?version=${GODOT_VERSION}&flavor=stable&slug=${GODOT_SLUG}.zip&platform=${GODOT_PLATFORM}")
FetchContent_Declare(
    godot 
    URL "${GODOT_URL}" 
    TIMEOUT 300               # 5-minute total limit
    INACTIVITY_TIMEOUT 30     # Fail if no data for 30 seconds
    TLS_VERIFY TRUE
)
FetchContent_MakeAvailable(godot)

# 3. Resolve the path once
file(GLOB GODOT_BIN_LIST "${godot_SOURCE_DIR}/Godot_v*")
list(FILTER GODOT_BIN_LIST EXCLUDE REGEX ".*\\.zip$")
list(GET GODOT_BIN_LIST 0 GODOT_ACTUAL_PATH)

set(GODOT_EXECUTABLE "${GODOT_ACTUAL_PATH}")
#####################################################[ (rclgd ) ]########################################################

find_package(ros_babel_fish REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Build godot-cpp
set(GODOTCPP_DISABLE_EXCEPTIONS OFF)
add_subdirectory(godot-cpp)

add_library(godot_cpp_no_cmake_warnings INTERFACE)
target_link_libraries(godot_cpp_no_cmake_warnings INTERFACE godot::cpp)
target_compile_options(godot_cpp_no_cmake_warnings INTERFACE -w)

# Extension Source
include_directories(src ${rclcpp_INCLUDE_DIRS} ${ros_babel_fish_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS})

set(SOURCES
    # RCLGD
    src/rclgd/rclgd.cpp
    src/rclgd/ros_msg.cpp
    src/rclgd/ros_node.cpp
    src/rclgd/ros_publisher.cpp
    src/rclgd/ros_subscriber.cpp
    src/rclgd/ros_client.cpp
    src/rclgd/ros_service.cpp
    src/rclgd/ros_tf_broadcaster.cpp
    src/rclgd/ros_tf_listener.cpp
    src/rclgd/ros_qos.cpp
    src/rclgd/ros_timer.cpp


    # TF2 Helpers
    src/tf2gd/ros_tf_broadcaster_3d.cpp
    src/tf2gd/ros_tf_listener_3d.cpp

    # Type Registration
    src/register_types.cpp

)

add_library(rclgd SHARED ${SOURCES})
ament_target_dependencies(rclgd rclcpp ros_babel_fish tf2_ros)
target_link_libraries(rclgd godot_cpp_no_cmake_warnings)



#####################################################[ INSTALLATION ]###############################################################

# Installation of godot and the extension in the same place
install(FILES "${GODOT_ACTUAL_PATH}"
        DESTINATION lib/${PROJECT_NAME}
        RENAME godot
        PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE 
                    GROUP_READ GROUP_EXECUTE 
                    WORLD_READ WORLD_EXECUTE)
install(TARGETS rclgd DESTINATION lib/${PROJECT_NAME})

# Install the colcon integration
ament_python_install_package(colcon_rclgd)
# Expose the shared library for godot packages to pick
ament_export_libraries(rclgd)
ament_package()