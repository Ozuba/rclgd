cmake_minimum_required(VERSION 3.8)
project(rclgd)

# Compiler options
add_compile_options("-O3")
find_package(ament_cmake REQUIRED)

#####################################################[ EDITOR & TOOLS ]###########################################################

# 1. Determine the Godot architecture suffix
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    set(ARCH_SUFFIX "x86_64")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
    set(ARCH_SUFFIX "arm64")
else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}.")
endif()

# 2. Godot Versioning & Binary Path
set(GODOT_VERSION "4.5.1")
set(GODOT_STATUS "stable")
set(GODOT_BIN "${CMAKE_BINARY_DIR}/godot")
set(GODOT_ZIP "${CMAKE_BINARY_DIR}/godot.zip")
set(GODOT_URL "https://github.com/godotengine/godot/releases/download/${GODOT_VERSION}-${GODOT_STATUS}/Godot_v${GODOT_VERSION}-${GODOT_STATUS}_linux.${ARCH_SUFFIX}.zip")

# 3. NATIVE DOWNLOAD RULE (Replaces install_godot.bash)
# This rule only executes if ${GODOT_BIN} is missing. 
# In a parallel build, CMake ensures only ONE thread executes this.
add_custom_command(
    OUTPUT "${GODOT_BIN}"
    COMMAND ${CMAKE_COMMAND} -E echo "Downloading Godot ${GODOT_VERSION}..."
    COMMAND curl -fSL "${GODOT_URL}" -o "${GODOT_ZIP}"
    COMMAND ${CMAKE_COMMAND} -E tar xf "${GODOT_ZIP}"
    # Find the versioned binary and rename it to 'godot' atomically
    COMMAND find . -maxdepth 1 -name "Godot_v*" -type f -not -name "*.zip" -exec mv {} "${GODOT_BIN}" \;
    COMMAND ${CMAKE_COMMAND} -E rm -f "${GODOT_ZIP}"
    COMMAND chmod +x "${GODOT_BIN}"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
    COMMENT "Fetching Godot Engine Binary for Headless Import"
    VERBATIM
)

# Drive the command with a target
add_custom_target(godot_binary_install DEPENDS "${GODOT_BIN}")

# 4. Filter Assets for Dependency Tracking
file(GLOB_RECURSE GODOT_ASSET_FILES
    RELATIVE "${CMAKE_SOURCE_DIR}/rclgd_demo"
    CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/rclgd_demo/*"
)
list(FILTER GODOT_ASSET_FILES EXCLUDE REGEX "^\\.godot/.*")

set(GODOT_ASSET_DEPENDENCIES)
foreach(FILE IN ITEMS ${GODOT_ASSET_FILES})
    list(APPEND GODOT_ASSET_DEPENDENCIES "${CMAKE_SOURCE_DIR}/rclgd_demo/${FILE}")
endforeach()

# 5. Resource Import Target
# Using --quit-after 20 to prevent "Scan thread aborted" errors
add_custom_target(
    godot_import_resources
    ALL
    COMMAND "${GODOT_BIN}" --editor --headless --quit-after 20 --path "${CMAKE_SOURCE_DIR}/rclgd_demo"
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    DEPENDS "${GODOT_BIN}" ${GODOT_ASSET_DEPENDENCIES}
    COMMENT "Importing Godot assets and baking shaders"
)

#####################################################[ SIM BUILD (rclgd) ]########################################################

find_package(ros_babel_fish REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Build godot-cpp
set(GODOTCPP_DISABLE_EXCEPTIONS OFF)
add_subdirectory(godot-cpp)

add_library(godot_cpp_no_cmake_warnings INTERFACE)
target_link_libraries(godot_cpp_no_cmake_warnings INTERFACE godot::cpp)
target_compile_options(godot_cpp_no_cmake_warnings INTERFACE -w)

# Extension Source
include_directories(src ${rclcpp_INCLUDE_DIRS} ${ros_babel_fish_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

set(SOURCES
    src/rclgd/rclgd.cpp
    src/rclgd/ros_msg.cpp
    src/rclgd/ros_node.cpp
    src/rclgd/ros_publisher.cpp
    src/rclgd/ros_subscriber.cpp
    src/register_types.cpp
)

add_library(rclgd SHARED ${SOURCES})
ament_target_dependencies(rclgd rclcpp ros_babel_fish)
target_link_libraries(rclgd godot_cpp_no_cmake_warnings)

# Copy Library to Extension Folder
add_custom_command(
    TARGET rclgd POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_SOURCE_DIR}/rclgd_demo/addons/rclgd/bin
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:rclgd> ${CMAKE_SOURCE_DIR}/rclgd_demo/addons/rclgd/bin/librclgd.so
    COMMENT "Deploying GDExtension library"
)

# ----------------------------- Hard Dependencies (Parallel Safety) -----------------------------

# Binary must exist before we can import
add_dependencies(godot_import_resources godot_binary_install)

# Library must be built/copied before import so Godot sees the new classes
add_dependencies(godot_import_resources rclgd)

#####################################################[ INSTALLATION ]###############################################################

install(PROGRAMS "${GODOT_BIN}" DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY rclgd_demo DESTINATION share/${PROJECT_NAME}/)
install(PROGRAMS scripts/launch_demo.sh DESTINATION lib/${PROJECT_NAME}/ RENAME rclgd_demo)

ament_package()