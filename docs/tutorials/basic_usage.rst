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

    func setup_subscription():
        sub = ros_node.create_subscription("/odom", "nav_msgs/msg/Odometry", _on_odom)

    func _on_odom(msg: RosMsg):
        # Access nested members directly
        var x = msg.pose.pose.position.x
        print("Robot X position: ", x)

.. note::
    Subscription callbacks are always invoked on the Godot main thread, so it is safe to touch the scene tree from them.
    ``create_subscriber`` is a deprecated alias of ``create_subscription``.

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
        if not client.wait_for_service(2.0):
            return

        var request = client.create_request()
        var future = client.async_send_request(request)
        var response = await future.completed
        if response:
            print("Status Success: ", response.success)

    # Server: fill the response in-place, it is sent when the callback returns
    func setup_service_server():
        var srv = ros_node.create_service("/get_status", "std_srvs/srv/Trigger", _handle_request)

    func _handle_request(_req: RosMsg, res: RosMsg):
        res.success = true
        res.message = "System OK"

.. warning::
    Service server callbacks run synchronously on the ROS executor thread (the response must be filled before returning to the client). Keep them self-contained and use ``call_deferred`` for side effects on the scene. See :ref:`class_RosService` for details.

Actions
-------

Actions are made for long-running tasks: they stream feedback while the goal executes and can be canceled mid-flight. Servers receive each accepted goal as a :ref:`class_RosServerGoalHandle` on the main thread and must finish it with ``succeed``, ``abort`` or ``canceled``.

.. code-block:: gdscript

    # Server: classic Fibonacci action
    func setup_action_server():
        var server = ros_node.create_action_server("fibonacci",
            "example_interfaces/action/Fibonacci", _execute_goal)

    func _execute_goal(goal_handle: RosServerGoalHandle):
        var sequence = [0, 1]
        for i in range(2, goal_handle.get_goal().order):
            if goal_handle.is_cancel_requested():
                var canceled_result = goal_handle.create_result()
                canceled_result.sequence = sequence
                goal_handle.canceled(canceled_result)
                return
            sequence.append(sequence[i - 1] + sequence[i - 2])
            var fb = goal_handle.create_feedback()
            fb.sequence = sequence
            goal_handle.publish_feedback(fb)
            # Spread long-running goals over frames instead of blocking
            await get_tree().process_frame
        var result = goal_handle.create_result()
        result.sequence = sequence
        goal_handle.succeed(result)

    # Client
    func send_fibonacci_goal():
        var client = ros_node.create_action_client("fibonacci",
            "example_interfaces/action/Fibonacci")
        if not client.wait_for_server(2.0):
            return

        var goal = client.create_goal()
        goal.order = 10
        var goal_handle = client.send_goal(goal)
        goal_handle.feedback.connect(func(msg): print("Partial: ", msg.sequence))

        await goal_handle.completed
        if goal_handle.get_status() == RosGoalHandle.STATUS_SUCCEEDED:
            print("Result: ", goal_handle.get_result().sequence)

A running goal can be canceled from the client with ``goal_handle.cancel()``; the final status then arrives through the ``completed`` signal as ``STATUS_CANCELED``.
