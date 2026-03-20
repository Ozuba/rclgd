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

Under the hood, **rclgd** runs the standard `rclcpp` executor in a background thread to manage network traffic without blocking the Godot main thread.

*   **Thread Safety**: Communication between the ROS 2 executor thread and the Godot Engine is carefully synchronized.
*   **Node Lifecycle**: The `RosNode` GDScript objects manage the lifetime of their underlying C++ `rclcpp::Node` counterparts.
*   **Initialization**: `rclcpp::init()` is invoked automatically when the Godot extension loads. You only need to initialize your specific `RosNode` instances inside your scripts.
*   **Memory Management**: Most **rclgd** classes inherit from `RefCounted`. They are automatically freed by Godot's memory manager when they fall out of scope, cleanly unregistering themselves from the ROS 2 graph.

Signals vs. Callbacks
---------------------

While Godot heavily relies on Signals for event handling, **rclgd** opts for `Callable` function references for high-frequency data streams (like topic subscriptions).

This approach drastically reduces the overhead of signal emission on the main thread for messages arriving at 100+ Hz. If you need standard signal behavior, you can easily emit a custom signal from within your subscription callback:

.. code-block:: gdscript

    signal message_received(data)

    func _on_ros_message(msg: RosMsg):
        # Safely emit a signal on the main thread for UI or other complex logic
        emit_signal("message_received", msg.get_member("data"))
