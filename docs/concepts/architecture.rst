Architecture & Integration
==========================

**rclgd** connects Godot's GDExtension system to ROS 2 (``rclcpp``), aiming for flexibility, performance and an idiomatic GDScript API.

BabelFish Integration
---------------------

Messages are handled at runtime by ``ros_babel_fish``, which reads each type's introspection data:

*   **No recompilation**: a new or custom interface package works as soon as it is built and sourced; the GDExtension never needs rebuilding.
*   **Type mapping**: fields map to Godot ``Variant`` types (numbers, strings, ``Packed*Array`` for numeric arrays, nested messages as objects, arrays of messages as arrays). Fields are converted lazily, on first access, and nested messages write straight into their parent's buffer.

Typed Message Classes
---------------------

On top of the dynamic core, **rclgd** registers a native engine class for every interface type the project depends on (see :doc:`/tutorials/custom_messages`):

*   **Discovery**: when the extension loads, the dependencies in ``res://package.xml`` are resolved through the ament index (``share/ament_index/resource_index/rosidl_interfaces/<pkg>`` on ``AMENT_PREFIX_PATH``). Every ``msg``, ``srv`` (request and response) and ``action`` (goal, result and feedback) type is registered, together with the types they nest. The set is fixed from then on; new types need a project reload.
*   **Registration**: each type (``geometry_msgs/msg/Twist``) becomes a class (``RosGeometryMsgsTwist``) deriving from :ref:`RosMsg<class_RosMsg>` in Godot's ``ClassDB``, with one typed property per field. Fields that clash with a ``RosMsg`` member or a GDScript keyword are left out (``get_member``/``set_member`` still reach them).
*   **Same core underneath**: a typed instance *is* a ``RosMsg`` (same buffer, same lazy conversion), and its properties are backed by ``get_member``/``set_member``.
*   **Wrapping**: every message rclgd hands to scripts (subscriptions, services, actions, nested fields, ``RosMsg.from_type``) is built by one factory that picks the typed class from a read-only table, so it is safe on the executor thread. Types without a class stay dynamic ``RosMsg`` instances.

Field access goes through Godot's property system, so it costs about as much as any native property. When reading several fields of a nested message, keep it in a local variable (``var p := msg.pose.position``) instead of repeating the chain.

Threading and Lifecycle
-----------------------

**rclgd** runs the standard ``rclcpp`` executor (a ``MultiThreadedExecutor``) in a background thread, so network traffic never blocks the Godot main thread. Launching with ``--ros-args -p use_separate_thread:=false`` switches to a single-threaded executor spun synchronously on the physics tick, for deterministic setups.

Where do callbacks run?

*   **Subscriptions, timers, action execute callbacks and action client signals**: dispatched to the Godot main thread (via ``call_deferred``), so plain GDScript is safe.
*   **Service server callbacks and action goal callbacks**: run synchronously on the executor thread, because ROS needs the answer before they return. Keep them self-contained and defer scene changes with ``call_deferred``.
*   In physics-synchronous mode everything runs on the main thread and no caveats apply.

Lifecycle details:

*   **Initialization**: ``rclgd.init()`` initializes the ROS 2 context and starts the executor; call it once before creating nodes. In projects created with ``ros2 rclgd create``, the ``ROS`` autoload does it (see :doc:`/tutorials/basic_usage`).
*   **Shutdown**: ``rclgd.shutdown()`` stops the executor and tears the context down. It also runs automatically when the extension unloads, so the process always exits cleanly. ``Ctrl+C`` (SIGINT) and SIGTERM (``ros2 launch`` shutdown, timeouts, systemd) are translated into a regular ``SceneTree.quit()`` on the main thread, so DDS participants are torn down cleanly.
*   **Memory management**: most **rclgd** classes are ``RefCounted``. They are freed when the last reference goes away, unregistering themselves from the ROS 2 graph. The flip side: keep a reference to every entity you create, or it disappears.
*   **Hidden node**: rclgd keeps one internal node per process, named ``rclgd_<pid>`` so several Godot apps can run side by side. It holds the process-level parameters (``use_separate_thread``, ``use_sim_time``, ``publish_sim_time``) and, when enabled, the ``/clock`` publisher.

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

Error Handling
--------------

rclgd reports errors (an unknown message type, an entity that fails to create, a parameter that can't be declared, ...) through Godot's error log, with a link to the C++ source. While the debugger is attached, they also break into the script debugger at the calling line. Set the ``rclgd/debug/break_on_error`` project setting to ``false`` to only log them.

Signals vs. Callbacks
---------------------

Godot relies heavily on signals, but **rclgd** takes ``Callable`` references for high-frequency streams such as subscriptions, which avoids the cost of signal emission for messages arriving at 100+ Hz. If you need signal behavior, emit your own signal from the callback:

.. code-block:: gdscript

    signal message_received(data)

    func _on_ros_message(msg: RosMsg):
        # Runs on the main thread, so connected UI code is safe
        message_received.emit(msg.data)
