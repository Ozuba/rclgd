Installation
============

This page describes how to install and setup **rclgd** in your Godot project.

Prerequisites
-------------

- Godot Engine 4.x (GDExtension support required).
- ROS2 (Humble, Iron, or Jazzy recommended).
- `ros_babel_fish` installed in your ROS2 environment.

Installation Steps
------------------

1. **Clone the repository**:
   Clone the `rclgd` repository into your ROS2 workspace `src` folder.

   .. code-block:: bash

      cd ~/ros2_ws/src
      git clone https://github.com/Ozuba/rclgd.git

2. **Build the package**:
   Use `colcon` to build the package.

   .. code-block:: bash

      cd ~/ros2_ws
      colcon build --packages-select rclgd

3. **Source the workspace**:

   .. code-block:: bash

      source install/setup.bash

4. **Add to Godot Project**:
    Create a godot project, you can use `rclgd Template <https://godotengine.org/asset-library/asset/4621>` or add your own
   `.gdextension` file as follows.
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

Binary gdextension will be loaded automatically from rclgd install folder.

Next Steps
----------

Now that you have installed rclgd, check out the :ref:`sec-tutorials` to create your first ROS2 Node in Godot.
