Creating an rclgd Package
=========================

**rclgd** provides a custom integration for **colcon**, the official ROS 2 build tool. This allows you to treat Godot projects as first-class ROS 2 packages, enabling seamless building, installation, and launching alongside your C++ and Python nodes.

The ``package.xml``
-------------------

To make a Godot project recognizable by ``colcon`` as an **rclgd** package, include a ``package.xml`` file in your project root with the custom build type:

.. code-block:: xml

    <?xml version="1.0"?>
    <package format="3">
      <name>my_godot_robot</name>
      <version>0.1.0</version>
      <description>My awesome Godot robot simulation</description>
      <maintainer email="me@example.com">Me</maintainer>
      <license>MIT</license>

      <buildtool_depend>rclgd</buildtool_depend>

      <export>
        <build_type>rclgd</build_type>
      </export>
    </package>

Key elements:

*   **build_type**: Must be set to ``rclgd``. This triggers the custom build logic.
*   **buildtool_depend**: Adding ``rclgd`` ensures that the build tool extension is available.

How Colcon Integration Works
----------------------------

When you run ``colcon build``, the **rclgd** build extension performs several specific tasks:

1.  **Project Synchronization**: It syncs your Godot source files to the ``install/`` directory. (Using ``--symlink-install`` creates a symbolic link, which is recommended for faster iteration).
2.  **Headless Asset Import**: It invokes the Godot editor in ``--headless`` mode to trigger the initial import of assets. This ensures resources are ready before launch.
3.  **Ament Indexing**: It registers the package in the Ament Resource Index, allowing tools like ``ros2 pkg prefix`` to locate your Godot project.
4.  **Launcher Shim**: It creates an executable script in ``install/lib/<package_name>/<package_name>``. This script automatically locates the Godot binary and runs it with the correct path to your project.

Launching Your Package
----------------------

Once built and sourced, you can launch your Godot project using standard ROS 2 commands:

.. code-block:: bash

    ros2 run my_godot_robot my_godot_robot

This will launch the Godot engine, load your project, and start your ROS 2 nodes defined in GDScript.
