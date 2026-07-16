Using Custom Messages
=====================

One of the most powerful features of **rclgd** is its native support for custom ROS 2 messages, without requiring you to recompile the GDExtension.

Dynamic Discovery
-----------------

Thanks to the underlying `ros_babel_fish` integration, **rclgd** interrogates your ROS 2 environment at runtime. As long as your custom message package is built and sourced in your current workspace overlay, **rclgd** can publish and subscribe to it natively.

Typed Wrappers (automatic)
--------------------------

For every message package listed as a dependency in your project's
``package.xml``, the rclgd editor plugin generates **typed GDScript wrappers**
(shadow classes) into ``res://addons/rclgd/gen`` — one class per message type,
named ``Ros`` + PascalCase (e.g. ``geometry_msgs/msg/Twist`` becomes
``RosGeometryMsgsTwist``). They give you autocompletion, typed property access
and type checking in the editor.

The workflow is entirely declarative:

1. Add the message package to your ``package.xml``:

   .. code-block:: xml

      <depend>my_msgs</depend>

2. Open (or reopen) the project in the editor — the wrappers appear
   automatically, including everything the types reference recursively
   (``Header``, geometry primitives, ...). To force a refresh at any time use
   **Project > Tools > Regenerate ROS2 Types**.

3. Commit ``addons/rclgd/gen`` with your project. The wrappers are plain
   GDScript files; they regenerate whenever the dependency set changes.

With the wrappers in place, messages are typed end to end — instances received
in callbacks automatically carry the matching class:

.. code-block:: gdscript

    var msg := RosMyMsgsRobotStatus.new()
    msg.name = "Titan-1"            # autocompleted, type-checked
    msg.battery_level = 85.5
    msg.position.x = 3.0            # nested types are typed too
    pub.publish(msg)

    func _on_status(msg: RosMyMsgsRobotStatus):
        print(msg.name, " at ", msg.battery_level, "%")

Dynamic access (no wrappers)
----------------------------

Wrappers are a convenience, not a requirement. Any type can be built and
inspected dynamically:

.. code-block:: gdscript

    extends Node

    var node: RosNode
    var pub: RosPublisher
    var demo_timer: RosTimer

    func _ready():
        node = RosNode.new()
        node.init("custom_msg_example")

        # Setup publisher for the custom message type
        pub = node.create_publisher("robot_status", "my_msgs/msg/RobotStatus")

        # Keep the returned reference! rclgd objects are RefCounted: a timer
        # whose return value is discarded is freed immediately and never fires.
        demo_timer = node.create_timer(2.0, _send_status)

    func _send_status():
        var msg = RosMsg.from_type("my_msgs/msg/RobotStatus")

        # Fields are regular properties, nested messages included
        msg.name = "Titan-1"
        msg.battery_level = 85.5
        msg.position.x = randf_range(-10.0, 10.0)
        msg.position.y = randf_range(-10.0, 10.0)
        msg.position.z = 0.0

        pub.publish(msg)

Manual generation
-----------------

The plugin is a thin layer over :ref:`RosMsg.gen_editor_support()<class_RosMsg_method_gen_editor_support>`,
which you can also call yourself — e.g. to generate wrappers for a type that
is not among your declared dependencies:

.. code-block:: gdscript

    RosMsg.gen_editor_support("my_msgs/msg/RobotStatus", "res://addons/rclgd/gen")
