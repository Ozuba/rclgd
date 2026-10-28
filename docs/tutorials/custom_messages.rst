Using Custom Messages
=====================

One of the most powerful features of **rclgd** is its native support for custom ROS 2 messages, without requiring you to recompile the GDExtension.

Dynamic Discovery
-----------------

Thanks to the underlying `ros_babel_fish` integration, **rclgd** interrogates your ROS 2 environment at runtime. As long as your custom message package is built and sourced in your current workspace overlay, **rclgd** can publish and subscribe to it natively.

Typed message classes (automatic)
---------------------------------

For every interface package listed as a dependency in your project's
``package.xml``, rclgd registers **native typed classes** when the extension
loads — one class per type, named ``Ros`` + PascalCase (e.g.
``geometry_msgs/msg/Twist`` becomes ``RosGeometryMsgsTwist``). They are real
engine classes deriving from :ref:`RosMsg<class_RosMsg>`: you get
autocompletion, typed properties, type checking and entries in the editor
help, and no files are generated.

Every interface kind is covered:

* messages: ``my_msgs/msg/RobotStatus`` → ``RosMyMsgsRobotStatus``
* service requests and responses: ``example_interfaces/srv/AddTwoInts`` →
  ``RosExampleInterfacesSrvAddTwoIntsRequest`` / ``...Response``
* action goals, results and feedback: ``example_interfaces/action/Fibonacci`` →
  ``RosExampleInterfacesActionFibonacciGoal`` / ``...Result`` / ``...Feedback``

The workflow is entirely declarative:

1. Add the interface package to your ``package.xml``:

   .. code-block:: xml

      <depend>my_msgs</depend>

2. (Re)start the editor or the game with your workspace sourced. The classes
   appear, including everything the types reference recursively (``Header``,
   geometry primitives, ...).

.. note::

   The set of typed classes is fixed when the extension loads. After adding a
   dependency or building a new interface package, reload the project
   (**Project > Reload Current Project**) to pick them up.

.. note::

   rclgd reads ``package.xml`` at startup in exported games too, but Godot
   only exports resources by default. Add ``package.xml`` to the export
   preset's **Resources > Filters to export non-resource files/folders**, or
   the exported game has no typed classes and scripts using them fail to load.

Messages are typed end to end — instances received in callbacks, service
requests and responses, and action goals, feedback and results all arrive as
the matching class:

.. code-block:: gdscript

    var msg := RosMyMsgsRobotStatus.new()
    msg.name = "Titan-1"            # autocompleted, type-checked
    msg.battery_level = 85.5
    msg.position.x = 3.0            # nested messages are typed too
    pub.publish(msg)

    func _on_status(msg: RosMyMsgsRobotStatus):
        print(msg.name, " at ", msg.battery_level, "%")

Arrays of messages are typed arrays (``Array[RosGeometryMsgsPose]``), and a
typed class can stand in for the type name when creating publishers and
subscriptions:

.. code-block:: gdscript

    var pub := node.create_publisher("cmd_vel", RosGeometryMsgsTwist)

A field whose name clashes with a :ref:`RosMsg<class_RosMsg>` member or a
GDScript keyword is not exposed as a property; read and write it with
``get_member()`` / ``set_member()``. Run Godot with ``--verbose`` to see which
fields were left out.

Upgrading from generated wrappers
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Earlier rclgd versions generated GDScript wrappers into
``res://addons/rclgd/gen`` from an editor plugin. Both are gone:

* delete ``addons/rclgd/gen``: the native classes replace it, keep the same
  names, and its ``class_name`` scripts clash with them;
* delete ``addons/rclgd/plugin.cfg`` and ``addons/rclgd/rclgd.gd``, and remove
  the plugin from ``[editor_plugins]`` in ``project.godot`` (it calls the
  removed ``RosMsg.gen_editor_support()``). The ``ROS`` autoload stays.

``ros2 rclgd doctor`` flags both leftovers.

Dynamic access (no typed classes)
---------------------------------

Typed classes are a convenience, not a requirement. Any type — including one
that is not among your dependencies — can be built and inspected dynamically:

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
