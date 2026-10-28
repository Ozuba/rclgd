Installation
============

There are two ways to get **rclgd**: install the released binary packages from
apt (recommended for most users) or build the suite from source. Either way you
do **not** need to install Godot yourself — rclgd pins the exact engine version
it was compiled against and provisions it for you (see :ref:`godot-setup`).

Prerequisites
-------------

*   **ROS 2** Jazzy, Kilted, Lyrical or Rolling. Each has its own branch;
    binary packages are available for Jazzy.
*   For building from source: a colcon workspace (e.g. ``~/ros2_ws``) and the
    build essentials for C++ ROS 2 packages (``colcon``, a compiler,
    ``rosdep``).

Install from apt (recommended)
------------------------------

rclgd is published to the ROS 2 build farm, so on a machine with the ROS 2 apt
repositories configured you can install the whole suite with a single command:

.. code-block:: bash

   sudo apt update
   sudo apt install ros-jazzy-rclgd

This pulls in all three packages (``ros-jazzy-rclgd``,
``ros-jazzy-colcon-rclgd`` and ``ros-jazzy-rclgd-cli``) together with the
pinned Godot editor binary. The install is complete out of the box: nothing to
provision, no extra step. Source your ROS 2 environment and verify:

.. code-block:: bash

   source /opt/ros/jazzy/setup.bash
   ros2 rclgd doctor

Building from Source
--------------------

Build from source on distributions without binary packages, to hack on rclgd
itself, or to target a different Godot version. The repository contains three
ROS 2 packages that are built together:

.. list-table::
   :widths: 25 20 55
   :header-rows: 1

   * - Package
     - Type
     - Contents
   * - ``rclgd``
     - ament_cmake
     - the GDExtension (``librclgd.so``), the Godot launcher and the engine version pin
   * - ``colcon_rclgd``
     - ament_python
     - the ``build_type: rclgd`` colcon extension that builds Godot projects as ROS 2 packages
   * - ``rclgd_cli``
     - ament_python
     - the ``ros2 rclgd`` command and the project template

1. **Clone the branch for your distribution** (``jazzy``, ``kilted``,
   ``lyrical`` or ``rolling``) with the ``godot-cpp`` submodule:

   .. code-block:: bash

      cd ~/ros2_ws/src
      git clone -b jazzy --recurse-submodules https://github.com/Ozuba/rclgd.git

2. **Install dependencies** (pulls in ``ros_babel_fish`` among others):

   .. code-block:: bash

      cd ~/ros2_ws
      rosdep install --from-paths src --ignore-src -y -r

3. **Build and source**:

   .. code-block:: bash

      colcon build --packages-up-to rclgd rclgd_cli colcon_rclgd
      source install/setup.bash

.. _godot-setup:

The Godot Editor binary
-----------------------

The engine binary is provisioned at **build time**: the ``rclgd`` build
downloads exactly the release ``librclgd.so`` was compiled against (verified
against a pinned SHA-512) and installs it as ``lib/rclgd/godot-bin``, next to
``librclgd.so``. There are no caches, environment variables or extra steps, and
an apt install ships the binary like any other file.

To verify everything is wired up:

.. code-block:: bash

   ros2 rclgd doctor

``doctor`` checks the ROS environment, the extension library, the launcher,
the version pin and the engine binary (and, run inside a project, the
project's setup), and prints a hint for anything that is off.

Targeting a different Godot version
-----------------------------------

rclgd supports exactly one engine version per build — the one its ``godot-cpp``
bindings were generated from. To change it:

1. Retarget the ``godot-cpp`` submodule to the branch/tag matching the desired
   version.
2. Edit ``GODOT_VERSION`` and the ``GODOT_SHA512_*`` pins in
   ``rclgd/CMakeLists.txt`` to the matching release (the sums come from the
   release's ``SHA512-SUMS.txt``).
3. Rebuild.

The build warns if the pinned version and the submodule's API version disagree.

Running the Godot Editor
------------------------

Always launch the editor through the sourced ROS 2 environment so
``librclgd.so`` and all runtime dependencies resolve:

.. code-block:: bash

   ros2 rclgd editor <package>   # open a built rclgd package's source project
   ros2 rclgd editor             # current directory, or the project manager
   ros2 run rclgd godot          # the raw engine binary, no project logic

How extension resolution works
------------------------------

Projects created with ``ros2 rclgd create`` ship a tiny
``addons/rclgd/bin/rclgd.gdextension`` that references ``librclgd.so`` by bare
name. Godot resolves it relative to the running executable, and since the
build installs the engine binary next to ``librclgd.so`` in the install
prefix, there is exactly **one** copy of the library on the machine and
every project uses it. No per-project binaries, no symlinks — packages stay
pure source and fully portable.

Next Steps
----------

With installation complete, create your first package with the
:doc:`ros2 rclgd command </getting_started/cli>` or head to the
:ref:`sec-tutorials`.
