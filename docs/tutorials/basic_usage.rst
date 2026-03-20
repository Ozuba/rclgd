Basic Usage
===========

This guide covers the fundamental building blocks of an **rclgd** application: nodes, communication patterns, and periodic tasks.

Initializing a Node
-------------------

The `RosNode` is the entry point for all ROS 2 communication. It should be initialized with a unique name and optional namespace.

.. code-block:: gdscript

    extends Node

    var ros_node: RosNode

    func _ready():
        if not rclgd.ok():
            rclgd.init()

        ros_node = RosNode.new()
        # init(name, namespace)
        ros_node.init("my_godot_node", "/my_robot")

        print("Node Name: ", ros_node.get_name())
        print("Namespace: ", ros_node.get_namespace())

Topics (Pub/Sub)
----------------

Topics allow for continuous data streams between nodes.

Publishing
~~~~~~~~~~

.. code-block:: gdscript

    var pub: RosPublisher

    func setup_publisher():
        pub = ros_node.create_publisher("/cmd_vel", "geometry_msgs/msg/Twist")
        
    func send_velocity(linear: float, angular: float):
        var msg = RosMsg.from_type("geometry_msgs/msg/Twist")
        msg.linear.x = linear
        msg.angular.z = angular
        pub.publish(msg)

Subscribing
~~~~~~~~~~~

.. code-block:: gdscript

    var sub: RosSubscriber

    func setup_subscriber():
        sub = ros_node.create_subscriber("/odom", "nav_msgs/msg/Odometry", _on_odom)

    func _on_odom(msg: RosMsg):
        # Access nested members directly
        var x = msg.pose.pose.position.x
        print("Robot X position: ", x)

Timers
------

Use `RosTimer` for periodic logic instead of Godot's built-in timers when synchronization with the ROS executor is required.

.. code-block:: gdscript

    var timer: RosTimer

    func setup_timer():
        # create_timer(seconds, callback)
        timer = ros_node.create_timer(1.0, _on_timer_timeout)

    func _on_timer_timeout():
        print("Periodic task executed at ROS time: ", ros_node.now())

    func stop_timer():
        if timer.is_ready():
            timer.cancel()

Services
--------

Services provide a request-response pattern.

.. code-block:: gdscript

    # Client (Async)
    func call_status_service():
        var client = ros_node.create_client("/get_status", "std_srvs/srv/Trigger")
        var request = RosMsg.from_type("std_srvs/srv/TriggerRequest")
        
        var response = await client.send_request(request)
        if response:
            print("Status Success: ", response.success)

    # Server
    func setup_service_server():
        var srv = ros_node.create_service("/get_status", "std_srvs/srv/Trigger", _handle_request)

    func _handle_request(_req: RosMsg) -> RosMsg:
        var res = RosMsg.from_type("std_srvs/srv/TriggerResponse")
        res.success = true
        res.message = "System OK"
        return res
