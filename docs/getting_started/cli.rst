The ros2 rclgd Command
======================

rclgd ships a ``ros2 rclgd`` command (provided by the ``rclgd_cli`` package)
that covers the whole lifecycle of a Godot-ROS package: scaffolding, opening
the editor, provisioning the engine and diagnosing broken setups.

.. code-block:: text

   ros2 rclgd create <name>    Scaffold a new rclgd package
   ros2 rclgd editor [pkg]     Open the Godot editor on a package's source project
   ros2 rclgd setup            Download the Godot editor binary matching this build
   ros2 rclgd list             List built rclgd packages
   ros2 rclgd doctor           Check the installation and report problems

Tab completion works for verbs, options and built package names.

create
------

Scaffolds a complete, buildable package from the bundled template: a
``package.xml`` with ``build_type: rclgd``, a minimal Godot project, the rclgd
editor addon and a demo publisher/subscriber scene.

.. code-block:: bash

   ros2 rclgd create my_bot
   colcon build --packages-select my_bot && source install/setup.bash
   ros2 rclgd editor my_bot     # edit
   ros2 run my_bot my_bot       # run

Useful options:

*   ``--dependencies geometry_msgs sensor_msgs`` — message packages the project
    uses; they become ``<exec_depend>`` entries, and the editor generates typed
    wrappers for them (see :doc:`/tutorials/custom_messages`).
*   ``--renderer gl_compatibility`` — for headless CI machines or weak GPUs
    (default: ``forward_plus``).
*   ``--maintainer-name`` / ``--maintainer-email`` — default to your git config.
*   ``--dry-run`` — print what would be created without writing anything.

Package names must be ``lower_snake_case`` to satisfy ROS 2 naming rules.

editor
------

Opens the Godot editor from the sourced ROS environment, which is what makes
``librclgd.so`` resolve.

.. code-block:: bash

   ros2 rclgd editor my_bot     # a built package: opens its *source* project
   ros2 rclgd editor            # inside a project: opens the current directory
   ros2 rclgd editor            # anywhere else: opens the project manager

Given a package name, the verb resolves the package through the ament index
and opens the **source** project — never the installed copy, where edits would
be silently overwritten by the next ``colcon build``.

``--import-only`` runs the headless asset import and exits, which is handy in
CI or after cloning a project whose ``.godot`` folder is (correctly) not
committed.

setup
-----

Downloads the pinned Godot release into the install prefix (next to
``librclgd.so``) and verifies its SHA-512 checksum against the official
release sums.

.. code-block:: bash

   ros2 rclgd setup             # no-op if the binary is already there
   ros2 rclgd setup --force     # re-download (e.g. after a corrupted download)

``--skip-checksum`` skips verification when the sums file is unreachable.

list
----

Lists every built rclgd package with its install prefix and source location:

.. code-block:: bash

   $ ros2 rclgd list
   my_bot	~/ros2_ws/install/my_bot	~/ros2_ws/src/my_bot

``--names-only`` prints one package name per line (script friendly).

doctor
------

Checks the whole chain — ROS environment, ``librclgd.so``, launcher, version
pin, engine binary (exists, runs, matches the pin) and, when run inside a
project, the project's extension wiring and import state. Every failure comes
with a hint:

.. code-block:: text

   $ ros2 rclgd doctor
   ✓ ROS_DISTRO=jazzy, default rmw
   ✓ rclgd sourced from ~/ros2_ws/install/rclgd
   ✓ extension library ~/ros2_ws/install/rclgd/lib/rclgd/librclgd.so
   ✓ godot launcher installed (ros2 run rclgd godot)
   ✓ version pin: Godot 4.7.1
   ✓ godot 4.7.1.stable.official at ~/ros2_ws/install/rclgd/lib/rclgd/godot-bin
   all checks passed
