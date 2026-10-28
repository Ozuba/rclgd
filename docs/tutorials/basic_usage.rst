Basic Usage
===========

This guide covers the building blocks of an **rclgd** application: initialization, nodes, logging, topics, QoS, timers, services and actions.

Initialization
--------------

The ROS 2 context must be initialized once before any node is created. Projects made with ``ros2 rclgd create`` do this for you: their ``ROS`` autoload (``addons/rclgd/ros_init.gd``) calls ``rclgd.init()`` before the first scene loads, passing it:

*   the command-line arguments, so ``--ros-args`` (remappings, ``-p`` parameters, log levels) work as with any ROS node;
*   every project setting under ``ros/parameters/``, as ``-p name:=value`` (a command-line ``-p`` for the same name wins).

It also calls ``rclgd.shutdown()`` when the game exits. Without the autoload, initialize the context yourself:

.. code-block:: gdscript

    if not rclgd.ok():
        rclgd.init(OS.get_cmdline_user_args())

Creating a Node
---------------

The ``RosNode`` is the entry point for all ROS 2 communication. Initialize it with a unique name and an optional namespace:

.. code-block:: gdscript

    extends Node

    var ros_node: RosNode

    func _ready():
        ros_node = RosNode.new()
        ros_node.init("my_godot_node", "/my_robot")  # init(name, namespace = "")

        print("Node Name: ", ros_node.get_name())
        print("Namespace: ", ros_node.get_namespace())

.. warning::
    All rclgd communication objects (``RosNode``, ``RosPublisher``,
    ``RosSubscriber``, ``RosTimer``, ``RosService``, ...) are ``RefCounted``:
    they live only as long as something holds a reference to them. Always
    store the value returned by a ``create_*`` factory in a member variable.
    One kept in a local variable is freed when the function returns and
    silently stops working.

Logging
-------

Use the node's logger instead of ``print`` when the output should reach ROS tooling. :ref:`class_RosLogger` has the same API as rclpy's logger: messages get the usual severity formatting on the console and are published on ``/rosout``, where ``rqt_console`` and ``ros2 bag`` can pick them up.

.. code-block:: gdscript

    var logger: RosLogger

    func _ready():
        # ... after ros_node.init(...)
        logger = ros_node.get_logger()
        logger.debug("Raw sensor value: %f" % value)
        logger.info("Simulation started")
        logger.warning("Wheel slip detected")
        logger.error("Lost connection to the controller")
        logger.fatal("Vehicle state is unrecoverable")

GDScript has no keyword arguments, so rclpy's logging filters are passed as a dictionary. As in rclpy, filter state is kept per call site:

.. code-block:: gdscript

    func _physics_process(_delta):
        logger.info("Contact detected", {"once": true})
        logger.warning("Speed limit exceeded", {"throttle_duration_sec": 1.0})
        logger.info("Tick", {"skip_first": true})

Messages below the logger's level are dropped (the log methods return ``false``). The default level is ``INFO``. Change it at runtime with :ref:`set_level<class_RosLogger_method_set_level>`, or at launch with ``--ros-args --log-level my_godot_node:=debug``. Use :ref:`get_child<class_RosLogger_method_get_child>` to give a subsystem its own logger name and level:

.. code-block:: gdscript

    var lidar_logger := logger.get_child("lidar")   # "my_godot_node.lidar"
    lidar_logger.set_level(RosLogger.DEBUG)

Code without a node, such as autoloads or editor tools, can use ``rclgd.get_logger("my_tool")``, which works like ``rclpy.logging.get_logger()`` and logs to the console only. The singleton also has ``set_logger_level``, ``get_logger_effective_level`` and ``get_logging_severity_from_string``, matching the ``rclpy.logging`` functions.

.. note::
    ``RosNode.get_logger()`` is only available after ``init()``.

Topics (Pub/Sub)
----------------

Topics carry continuous data streams between nodes. Messages are created from their type: with the typed class when the interface package is a dependency in your ``package.xml`` (see :doc:`/tutorials/custom_messages`), or with ``RosMsg.from_type()`` for any type.

Publishing
~~~~~~~~~~

.. code-block:: gdscript

    var pub: RosPublisher

    func setup_publisher():
        pub = ros_node.create_publisher("/cmd_vel", "geometry_msgs/msg/Twist")

    func send_velocity(linear: float, angular: float):
        var msg := RosGeometryMsgsTwist.new()  # or RosMsg.from_type("geometry_msgs/msg/Twist")
        msg.linear.x = linear
        msg.angular.z = angular
        pub.publish(msg)

Subscribing
~~~~~~~~~~~

.. code-block:: gdscript

    var sub: RosSubscriber

    func setup_subscription():
        sub = ros_node.create_subscription("/odom", "nav_msgs/msg/Odometry", _on_odom)

    func _on_odom(msg: RosNavMsgsOdometry):  # or msg: RosMsg
        print("Robot X position: ", msg.pose.pose.position.x)

.. note::
    Subscription callbacks are always invoked on the Godot main thread, so it is safe to touch the scene tree from them.

Quality of Service
~~~~~~~~~~~~~~~~~~

Publishers, subscriptions, services and actions take an optional :ref:`class_RosQoS` as their last argument. Without one they use the ROS 2 defaults (reliable, volatile, keep last 10).

.. code-block:: gdscript

    func setup_scan_subscription():
        var qos := RosQoS.new()
        qos.reliability = RosQoS.BEST_EFFORT   # match typical sensor publishers
        qos.depth = 5
        sub = ros_node.create_subscription("/scan", "sensor_msgs/msg/LaserScan", _on_scan, qos)

For a "latched" topic, whose last message reaches late subscribers, set ``durability = RosQoS.TRANSIENT_LOCAL`` on both ends. Publisher and subscription profiles must be compatible, or no messages flow.

Timers
------

Use ``RosTimer`` for periodic logic that should follow the node's clock: with ``use_sim_time:=true`` timers follow ``/clock`` and pause when the simulation pauses.

.. code-block:: gdscript

    var timer: RosTimer

    func setup_timer():
        timer = ros_node.create_timer(1.0, _on_timer_timeout)  # create_timer(seconds, callback)

    func _on_timer_timeout():
        print("Periodic task at ROS time: ", ros_node.now().sec)

    func stop_timer():
        timer.cancel()

Timer callbacks run on the Godot main thread.

Services
--------

Services provide a request-response pattern.

.. code-block:: gdscript

    var client: RosClient
    var srv: RosService

    # Client: requests are awaitable
    func call_status_service():
        client = ros_node.create_client("/get_status", "std_srvs/srv/Trigger")
        if not client.wait_for_service(2.0):
            return

        var request := client.create_request()
        var response: RosMsg = await client.async_send_request(request).completed
        if response:
            print("Status Success: ", response.success)

    # Server: fill the response in place; it is sent when the callback returns
    func setup_service_server():
        srv = ros_node.create_service("/get_status", "std_srvs/srv/Trigger", _handle_request)

    func _handle_request(_req: RosMsg, res: RosMsg):
        res.success = true
        res.message = "System OK"

``wait_for_service`` blocks the main thread for up to the given timeout. To keep the game responsive, call ``wait_for_service_async`` instead and react to the ``service_available`` signal.

.. warning::
    Service server callbacks run synchronously on a ROS executor thread (the response must be filled before returning to the client). Keep them self-contained and use ``call_deferred`` for side effects on the scene. See :ref:`class_RosService` for details.

Actions
-------

Actions are made for long-running tasks: they stream feedback while the goal executes and can be canceled mid-flight. Servers receive each accepted goal as a :ref:`class_RosServerGoalHandle` on the main thread and must finish it with ``succeed``, ``abort`` or ``canceled``.

Goals are accepted by default. To reject some, pass a goal callback as the fourth argument of ``create_action_server``; it receives the goal message and returns ``true`` to accept. Cancel requests are always accepted, and the execute callback decides how to react through ``is_cancel_requested()``.

.. warning::
    The goal callback runs synchronously on a ROS executor thread, because ROS needs the answer immediately. Only inspect the goal message there and don't touch the scene tree. For checks that need game state, accept the goal and call ``abort`` at the start of the execute callback.

.. code-block:: gdscript

    var server: RosActionServer
    var action_client: RosActionClient

    # Server: classic Fibonacci action
    func setup_action_server():
        server = ros_node.create_action_server("fibonacci",
            "example_interfaces/action/Fibonacci", _execute_goal)

    func _execute_goal(goal_handle: RosServerGoalHandle):
        var sequence := PackedInt32Array([0, 1])
        for i in range(2, goal_handle.get_goal().order):
            if goal_handle.is_cancel_requested():
                var canceled_result := goal_handle.create_result()
                canceled_result.sequence = sequence
                goal_handle.canceled(canceled_result)
                return
            sequence.append(sequence[i - 1] + sequence[i - 2])
            var fb := goal_handle.create_feedback()
            fb.sequence = sequence
            goal_handle.publish_feedback(fb)
            # Spread long-running goals over frames instead of blocking
            await get_tree().process_frame
        var result := goal_handle.create_result()
        result.sequence = sequence
        goal_handle.succeed(result)

    # Client
    func send_fibonacci_goal():
        action_client = ros_node.create_action_client("fibonacci",
            "example_interfaces/action/Fibonacci")
        if not action_client.wait_for_server(2.0):
            return

        var goal := action_client.create_goal()
        goal.order = 10
        var goal_handle := action_client.send_goal(goal)
        goal_handle.feedback.connect(func(msg): print("Partial: ", msg.sequence))

        await goal_handle.completed
        if goal_handle.get_status() == RosGoalHandle.STATUS_SUCCEEDED:
            print("Result: ", goal_handle.get_result().sequence)

A running goal can be canceled from the client with ``goal_handle.cancel()``; the final status then arrives through the ``completed`` signal as ``STATUS_CANCELED``. Like ``wait_for_service``, ``wait_for_server`` blocks; ``wait_for_server_async`` and the ``server_available`` signal are the non-blocking alternative.
