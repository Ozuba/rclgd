Using Custom Messages
=====================

One of the most powerful features of **rclgd** is its support for custom ROS2 messages without requiring you to recompile the GDExtension.

Dynamic Discovery
-----------------

Thanks to `ros_babel_fish`, `rclgd` discovers messages available in your ROS2 environment at runtime. As long as your custom message package is built and sourced in your workspace, `rclgd` can use it.

Full Example
------------

Here is how you would use a custom message in a complete script.

.. code-block:: gdscript

    extends Node

    var node: RosNode
    var pub: RosPublisher

    func _ready():
        node = RosNode.new()
        node.init("custom_msg_example")
        
        # Setup publisher for custom message
        pub = node.create_publisher("robot_status", "my_msgs/msg/RobotStatus")
        
        # GENERATE AUTOCOMPLETE (Optional, run once in editor)
        # RosMsg.gen_editor_support("my_msgs/msg/RobotStatus", "res://ros_msgs/")
        
        var timer = Timer.new()
        add_child(timer)
        timer.wait_time = 2.0
        timer.timeout.connect(_send_status)
        timer.start()

    func _send_status():
        var msg = RosMsg.from_type("my_msgs/msg/RobotStatus")
        
        msg.set_member("name", "Titan-1")
        msg.set_member("battery_level", 85.5)
        
        # Accessing nested messages
        var pos = msg.get_member("position")
        pos.set_member("x", randf_range(-10, 10))
        pos.set_member("y", randf_range(-10, 10))
        pos.set_member("z", 0.0)
        
        pub.publish(msg)
        print("Status sent!")

Editor Autocomplete
-------------------

To help with development, `rclgd` can generate helper scripts that provide autocomplete support for your message types.

.. code-block:: gdscript

    RosMsg.gen_editor_support("my_msgs/msg/RobotStatus", "res://ros_msgs/")

This will generate a GDScript file in ``res://ros_msgs/`` with properties matching your message fields, allowing you to use typed access in the Godot Editor.
