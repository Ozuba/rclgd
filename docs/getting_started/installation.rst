Installation
============

This guide explains how to install and configure **rclgd** for your Godot 4 project.

Prerequisites
-------------

*   **Godot Engine 4.x** (Standard version with GDExtension support).
*   **ROS 2** (Humble, Iron, or Jazzy recommended).
*   **ros_babel_fish**: Must be installed in your ROS 2 environment (e.g., `sudo apt install ros-<distro>-ros-babel-fish`).

Building from Source
--------------------

**rclgd** is built using `colcon`, integrating seamlessly into your existing ROS 2 workspace.

1. **Clone the repository**:
   Navigate to your workspace's source directory and clone the repository.

   .. code-block:: bash

      cd ~/ros2_ws/src
      git clone https://github.com/Ozuba/rclgd.git

2. **Build the package**:
   Compile the extension.

   .. code-block:: bash

      cd ~/ros2_ws
      colcon build --packages-select rclgd

3. **Source the workspace**:
   Make the built libraries available to your environment.

   .. code-block:: bash

      source install/setup.bash

Integrating with Godot
----------------------

To use **rclgd** in an existing Godot project, you must register the GDExtension.

1. Create a file named `rclgd.gdextension` in your Godot project root (or a dedicated `bin/` folder).
2. Add the following configuration to point the engine to the built libraries:

.. code-block:: cfg

    [configuration]
    entry_symbol = "rclgd_init"
    compatibility_minimum = "4.3"
    reloadable = false

    [libraries]
    linux.editor.x86_64 = "librclgd.so"
    linux.debug.x86_64 = "librclgd.so"
    linux.release.x86_64 = "librclgd.so"

    linux.editor.arm64 = "librclgd.so"
    linux.debug.arm64 = "librclgd.so"
    linux.release.arm64 = "librclgd.so"

*Note: Since standard `colcon build` makes the library available in the environment path, Godot will dynamically resolve `librclgd.so` from your sourced ROS 2 installation folder.*

Next Steps
----------

With installation complete, proceed to the :ref:`sec-tutorials` to initialize your first Node.
