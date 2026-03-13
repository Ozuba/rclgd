Creating an rclgd Package
=========================

**rclgd** provides a custom integration for **colcon**, the ROS2 build tool. This allows you to treat Godot projects as first-class ROS2 packages, enabling seamless building, installation, and launching alongside your C++/Python nodes.

The ``package.xml``
-------------------

To make a Godot project recognizable by ``colcon`` as an ``rclgd`` package, you must include a ``package.xml`` file in your project root with the custom build type:

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

When you run ``colcon build``, the **rclgd** build extension performs several tasks:

1.  **Project Synchronization**: It syncs your Godot source files to the ``install/`` directory. If you use ``--symlink-install``, it creates a symbolic link, which is highly recommended for faster iteration.
2.  **Headless Asset Import**: It invokes the Godot editor in ``--headless`` mode to trigger the initial import of assets (textures, meshes, etc.). This ensures that all resources are ready before you even launch the project.
3.  **Ament Indexing**: It registers the package in the Ament Resource Index, allowing tools like ``ros2 pkg prefix`` or ``ros2 share`` to locate your Godot project.
4.  **Launcher Shim**: It creates an executable script in ``install/lib/<package_name>/<package_name>``. This script automatically locates the Godot binary from the ``rclgd`` extension and runs it with the correct path to your project.

The Export Process
------------------

When you "build" a Godot package with ``colcon``, you are essentially preparing it for execution in a ROS2-native environment.

*   **Source Folder**: Your project root (contains ``project.godot``, ``package.xml``, etc.).
*   **Install Folder**:
    *   ``share/<package_name>``: Contains your project files.
    *   ``lib/<package_name>/<package_name>``: The executable shim.

Launching Your Package
----------------------

Once built and sourced, you can launch your Godot project using standard ROS2 commands:

.. code-block:: bash

    ros2 run my_godot_robot my_godot_robot

This will launch the Godot engine, which will then load your project and start your ROS2 nodes defined in GDScript.

Benefits of this Approach
-------------------------

*   **Consistency**: Manage your engine-based nodes exactly like your C++ nodes.
*   **Dependency Management**: Use ``package.xml`` to declare dependencies on other ROS2 message packages.
*   **Environment Sourcing**: Godot projects automatically inherit the ROS2 environment, ensuring all message types and plugins are available.
*   **CI/CD Ready**: Since building and importing is handled by ``colcon``, you can easily integrate Godot simulations into automated testing pipelines.
