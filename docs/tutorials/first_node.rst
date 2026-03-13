Your First ROS2 Node in Godot
=============================

This tutorial will guide you through creating your first ROS2 Node using **rclgd** in Godot.

Creating the Node
-----------------

In GDScript, you can create a ROS2 node by instantiating the `RosNode` class.

.. code-block:: gdscript

    extends Node

    var ros_node: RosNode

    func _ready():
        # Initialize the ROS2 node
        ros_node = RosNode.new()
        ros_node.init("my_godot_node")
        
        print("ROS2 Node initialized: ", ros_node.get_name())

Spinning the Node
-----------------

Unlike standard ROS2 applications, `rclgd` handles spinning internally via the GDExtension lifecycle, or you can manually poll if needed. Usually, once initialized, the node is active in the ROS2 graph.

Checking the Node
-----------------

Run your Godot project, and in a terminal, run:

.. code-block:: bash

    ros2 node list

You should see `/my_godot_node` in the list.
