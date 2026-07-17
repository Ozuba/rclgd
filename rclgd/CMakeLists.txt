cmake_minimum_required(VERSION 3.18)
project(rclgd)

# Optimized by default; buildfarm/CI overrides via -DCMAKE_BUILD_TYPE
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
find_package(ament_cmake REQUIRED)
#####################################################[ GODOT RUNTIME ]##########################################################
# The engine binary is downloaded at build time and installed as
# lib/rclgd/godot-bin — next to librclgd.so, where GDExtension resolves the
# bare "librclgd.so" path in rclgd.gdextension. It ships with the package, so
# an apt/deb install is complete out of the box and the file is owned by the
# package manager. The archive is verified against a pinned SHA512 before use.

# The one supported engine version: the version the godot-cpp submodule (and
# therefore librclgd.so) is built against. To target another version,
# retarget the submodule, edit this line, and refresh the SHA512 pins below
# from the release's SHA512-SUMS.txt — plain variables (not CACHE), so edits
# always take effect on rebuild.
set(GODOT_VERSION "4.7.1")
set(GODOT_SHA512_LINUX_X86_64 "4ccdab7a48eeccbe8819a2fc1f6262f8d72065d98601bcb3743fcbd7ebd39f373758a788ee3293a05ec5b2c48538266c437404312e372225cd2df273945a2de9")
set(GODOT_SHA512_LINUX_ARM64 "de64efe4d936ac0403769e078a73d961a9c647cab04168c5fb5a7fe33728e200a67324ed99368eeb27964e205e72a61e48efb63b52d5de34d12dd6a95ca0fc45")
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
# Version pin, kept for the CLI (`ros2 rclgd doctor`/`editor`) to report
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/godot_version" "${GODOT_VERSION}\n")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/godot_version" DESTINATION share/${PROJECT_NAME})

# Download the official release archive for the target architecture and
# verify it. file(DOWNLOAD) skips the fetch when the file already exists
# with a matching hash, so only clean rebuilds hit the network.
if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(FATAL_ERROR "rclgd currently targets Linux (got ${CMAKE_SYSTEM_NAME})")
endif()
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    set(_godot_slug "linux.x86_64")
    set(_godot_sha512 "${GODOT_SHA512_LINUX_X86_64}")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
    set(_godot_slug "linux.arm64")
    set(_godot_sha512 "${GODOT_SHA512_LINUX_ARM64}")
else()
    message(FATAL_ERROR "no Godot release for architecture ${CMAKE_SYSTEM_PROCESSOR}")
endif()
set(_godot_name "Godot_v${GODOT_VERSION}-stable_${_godot_slug}")
set(_godot_zip "${CMAKE_CURRENT_BINARY_DIR}/${_godot_name}.zip")
message(STATUS "rclgd: providing Godot ${GODOT_VERSION} (${_godot_slug})")
file(DOWNLOAD
    "https://github.com/godotengine/godot/releases/download/${GODOT_VERSION}-stable/${_godot_name}.zip"
    "${_godot_zip}"
    EXPECTED_HASH SHA512=${_godot_sha512}
    INACTIVITY_TIMEOUT 60
    STATUS _godot_dl_status)
list(GET _godot_dl_status 0 _godot_dl_code)
if(NOT _godot_dl_code EQUAL 0)
    list(GET _godot_dl_status 1 _godot_dl_msg)
    message(FATAL_ERROR
        "rclgd: downloading Godot ${GODOT_VERSION} failed: ${_godot_dl_msg}\n"
        "The build needs network access to fetch the engine binary.")
endif()
file(ARCHIVE_EXTRACT INPUT "${_godot_zip}" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/godot-engine")
if(NOT EXISTS "${CMAKE_CURRENT_BINARY_DIR}/godot-engine/${_godot_name}")
    message(FATAL_ERROR "rclgd: unexpected layout in ${_godot_name}.zip")
endif()
install(PROGRAMS "${CMAKE_CURRENT_BINARY_DIR}/godot-engine/${_godot_name}"
    DESTINATION lib/${PROJECT_NAME} RENAME godot-bin)

# Launcher: exec's godot-bin next to itself. Installed as lib/rclgd/godot so
# `ros2 run rclgd godot` works.
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