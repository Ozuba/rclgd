Using Custom Messages
=====================

One of the most powerful features of **rclgd** is its native support for custom ROS 2 messages, without requiring you to recompile the GDExtension.

Dynamic Discovery
-----------------

Thanks to the underlying `ros_babel_fish` integration, **rclgd** interrogates your ROS 2 environment at runtime. As long as your custom message package is built and sourced in your current workspace overlay, **rclgd** can publish and subscribe to it natively.

Full Example
------------

Here is how you use a custom message in GDScript. Note that nested message fields are fully supported.

.. code-block:: gdscript

    extends Node

    var node: RosNode
    var pub: RosPublisher

    func _ready():
        node = RosNode.new()
        node.init("custom_msg_example")
        
        # Setup publisher for the custom message type
        pub = node.create_publisher("robot_status", "my_msgs/msg/RobotStatus")
        
        var timer = Timer.new()
        add_child(timer)
        timer.wait_time = 2.0
        timer.timeout.connect(_send_status)
        timer.start()

    func _send_status():
        var msg = RosMsg.from_type("my_msgs/msg/RobotStatus")
        
        msg.set_member("name", "Titan-1")
        msg.set_member("battery_level", 85.5)
        
        # Accessing nested messages dynamically
        var pos = msg.get_member("position")
        pos.set_member("x", randf_range(-10.0, 10.0))
        pos.set_member("y", randf_range(-10.0, 10.0))
        pos.set_member("z", 0.0)
        
        pub.publish(msg)

Editor Autocomplete
-------------------

To assist with GDScript development, **rclgd** provides a utility to generate helper scripts that offer autocomplete suggestions and typed properties in the Godot Editor.

.. code-block:: gdscript

    func generate_helpers():
        RosMsg.gen_editor_support("my_msgs/msg/RobotStatus", "res://ros_msgs/")

Running this function once will generate a GDScript wrapper file in ``res://ros_msgs/``. Including this script in your project allows you to use strongly-typed property access for your specific custom messages instead of `.set_member()`.
