Architecture & Integration
==========================

**rclgd** bridges the gap between Godot Engine's GDExtension system and ROS 2, focusing on flexibility, performance, and an idiomatic GDScript API.

BabelFish Integration
---------------------

A core component of **rclgd** is `ros_babel_fish`. This library enables dynamic runtime processing of ROS 2 messages.

*   **No Recompilation**: You do not need to recompile the GDExtension every time you introduce a new custom ROS 2 message type to your workspace.
*   **Type Mapping**: **rclgd** automatically maps ROS 2 primitive types to Godot `Variant` types. Complex structures (nested messages, arrays) are parsed and exposed recursively.

Threading and Lifecycle
-----------------------

Under the hood, **rclgd** runs the standard `rclcpp` executor in a background thread (a `MultiThreadedExecutor`) to manage network traffic without blocking the Godot main thread. Launching with ``--ros-args -p use_separate_thread:=false`` switches to a single-threaded executor spun synchronously on the physics tick, for deterministic setups.

Where do callbacks run?

*   **Subscriptions, timers, action clients and action servers**: always dispatched to the Godot main thread (via ``call_deferred``), so plain GDScript is safe.
*   **Service servers**: the callback runs synchronously on the executor thread, because the response must be filled before it is returned to the client. Keep these callbacks self-contained, or use an action server for long-running work.
*   In physics-synchronous mode everything runs on the main thread and no caveats apply.

Lifecycle details:

*   **Node Lifecycle**: The `RosNode` GDScript objects manage the lifetime of their underlying C++ `rclcpp::Node` counterparts.
*   **Initialization**: ``rclgd.init()`` initializes the ROS 2 context and starts the executor; call it once before creating nodes.
*   **Shutdown**: ``rclgd.shutdown()`` stops the executor and tears the context down. It is also invoked automatically when the extension unloads, so the process always exits cleanly. ``Ctrl+C`` (SIGINT) and SIGTERM (``ros2 launch`` shutdown, timeouts, systemd) are caught and translated into a regular ``SceneTree.quit()`` on the main thread, so DDS participants are always torn down cleanly.
*   **Memory Management**: Most **rclgd** classes inherit from `RefCounted`. They are automatically freed by Godot's memory manager when they fall out of scope, cleanly unregistering themselves from the ROS 2 graph. The flip side: keep a reference to every entity you create, or it disappears (see the warning in :doc:`/tutorials/basic_usage`).
*   **Hidden node**: rclgd maintains one internal node per process, named ``rclgd_<pid>`` so several Godot apps can run side by side. It carries the process-level parameters below and, when enabled, the ``/clock`` publisher.

Simulation Time
---------------

rclgd separates the two roles around ``/clock``:

*   ``--ros-args -p use_sim_time:=true`` — standard ROS semantics: node clocks
    and :ref:`RosTimer<class_RosTimer>` follow ``/clock``. Use this alone in a
    Godot app that runs next to an external simulator (Gazebo, another Godot
    instance).
*   ``--ros-args -p publish_sim_time:=true`` — Godot **is** the time source:
    the physics tick drives ``/clock``, scaled by ``Engine.time_scale`` (a
    time scale of 0 pauses the clock).

A Godot app that is itself the simulator typically passes both.

Signals vs. Callbacks
---------------------

While Godot heavily relies on Signals for event handling, **rclgd** opts for `Callable` function references for high-frequency data streams (like topic subscriptions).

This approach drastically reduces the overhead of signal emission on the main thread for messages arriving at 100+ Hz. If you need standard signal behavior, you can easily emit a custom signal from within your subscription callback:

.. code-block:: gdscript

    signal message_received(data)

    func _on_ros_message(msg: RosMsg):
        # Safely emit a signal on the main thread for UI or other complex logic
        emit_signal("message_received", msg.get_member("data"))
