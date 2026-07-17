Creating an rclgd Package
=========================

**rclgd** provides a custom integration for **colcon**, the official ROS 2 build tool. This allows you to treat Godot projects as first-class ROS 2 packages, enabling seamless building, installation, and launching alongside your C++ and Python nodes.

The fast way: ``ros2 rclgd create``
-----------------------------------

The CLI scaffolds a complete, buildable package in one command:

.. code-block:: bash

    ros2 rclgd create my_godot_robot --dependencies geometry_msgs
    colcon build --packages-select my_godot_robot
    source install/setup.bash

The generated package contains:

.. code-block:: text

    my_godot_robot/
    ├── package.xml          # ROS 2 manifest, build_type: rclgd
    ├── project.godot        # Godot project (named after the package)
    ├── main.tscn / main.gd  # demo scene: a publisher + subscriber on /chatter
    ├── .gitignore           # ignores .godot/ import artifacts
    └── addons/rclgd/        # the rclgd editor addon
        ├── plugin.cfg
        ├── rclgd.gd         # editor plugin: generates typed message wrappers
        ├── ros_init.gd      # autoload: initializes the ROS context at game start
        └── bin/rclgd.gdextension

From here, ``ros2 rclgd editor my_godot_robot`` opens the project and
``ros2 run my_godot_robot my_godot_robot`` runs it as a regular ROS 2
executable. See :doc:`/getting_started/cli` for all options.

The ``package.xml``
-------------------

If you are converting an existing Godot project by hand, the only requirement
is a ``package.xml`` in the project root with the custom build type:

.. code-block:: xml

    <?xml version="1.0"?>
    <package format="3">
      <name>my_godot_robot</name>
      <version>0.1.0</version>
      <description>My awesome Godot robot simulation</description>
      <maintainer email="me@example.com">Me</maintainer>
      <license>MIT</license>

      <buildtool_depend>rclgd</buildtool_depend>
      <depend>geometry_msgs</depend>

      <export>
        <build_type>rclgd</build_type>
      </export>
    </package>

Key elements:

*   **build_type**: Must be set to ``rclgd``. This triggers the custom build logic.
*   **buildtool_depend**: Adding ``rclgd`` ensures that the build tool extension is available.
*   **depend**: Message packages your project uses. Besides declaring the
    runtime dependency, these drive the automatic generation of typed GDScript
    wrappers in the editor (see :doc:`/tutorials/custom_messages`).

You will also need the ``addons/rclgd`` folder — copy it from a freshly
created package (``ros2 rclgd create``).

How Colcon Integration Works
----------------------------

When you run ``colcon build``, the **rclgd** build extension performs several specific tasks:

1.  **Project Synchronization**: It syncs your Godot source files to the ``install/`` directory. (Using ``--symlink-install`` creates a symbolic link, which is recommended for faster iteration).
2.  **Ament Indexing**: It registers the package in the Ament Resource Index — this is how ``ros2 pkg prefix``, ``ros2 rclgd list`` and ``ros2 rclgd editor`` locate your project.
3.  **Launcher Shim**: It creates an executable script in ``install/lib/<package_name>/<package_name>``. This script locates the Godot binary through the sourced environment and runs it with the correct project path, forwarding any ``--ros-args`` you pass on the command line.
4.  **Headless Asset Import**: It invokes the Godot editor in ``--headless`` mode to trigger the initial import of assets, so resources are ready before the first launch. (This step is skipped with a warning if the engine binary is missing — rebuild ``rclgd``, which installs it.)

Launching Your Package
----------------------

Once built and sourced, you can launch your Godot project using standard ROS 2 commands:

.. code-block:: bash

    ros2 run my_godot_robot my_godot_robot

ROS arguments work exactly as with any other node — the shim forwards them to
the ROS context inside Godot:

.. code-block:: bash

    ros2 run my_godot_robot my_godot_robot --ros-args -r __ns:=/robot1 -p use_sim_time:=true

This also means rclgd packages can be started from launch files like any other
executable.
