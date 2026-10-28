Using Custom Messages
=====================

**rclgd** supports any ROS 2 interface, custom ones included, without recompiling the GDExtension. Messages are resolved at runtime through ``ros_babel_fish``: as long as the interface package is built and sourced in your environment, you can publish, subscribe, call and serve it.

On top of that, the interface packages your project depends on get **native typed classes**, with autocompletion and type checking in the editor.

Typed message classes
---------------------

For every interface package listed as a dependency in your project's ``package.xml``, rclgd registers one native class per type when the extension loads. The class name is ``Ros`` + the PascalCase type name:

* messages: ``my_msgs/msg/RobotStatus`` → ``RosMyMsgsRobotStatus``
* service requests and responses: ``example_interfaces/srv/AddTwoInts`` →
  ``RosExampleInterfacesSrvAddTwoIntsRequest`` / ``...Response``
* action goals, results and feedback: ``example_interfaces/action/Fibonacci`` →
  ``RosExampleInterfacesActionFibonacciGoal`` / ``...Result`` / ``...Feedback``

Types those reference (``Header``, geometry primitives, ...) are registered too. The classes derive from :ref:`RosMsg<class_RosMsg>`, have one typed property per field, appear in the editor help, and no files are generated.

To use them:

1. Add the interface package to your ``package.xml``:

   .. code-block:: xml

      <depend>my_msgs</depend>

2. Reload the project (**Project > Reload Current Project**), or restart the game, with the workspace sourced.

.. note::

   The set of typed classes is fixed when the extension loads, so new dependencies and newly built interface packages only show up after a reload.

Messages are typed end to end. Instances you create, messages received in callbacks, service requests and responses, and action goals, feedback and results all arrive as the matching class:

.. code-block:: gdscript

    var msg := RosMyMsgsRobotStatus.new()
    msg.name = "Titan-1"            # autocompleted, type-checked
    msg.battery_level = 85.5
    msg.position.x = 3.0            # nested messages are typed too
    pub.publish(msg)

    func _on_status(msg: RosMyMsgsRobotStatus):
        print(msg.name, " at ", msg.battery_level, "%")

A few more details:

* Arrays of messages are typed arrays, e.g. ``Array[RosGeometryMsgsPose]``.
* A typed class can stand in for the type name when creating publishers and subscriptions: ``node.create_publisher("cmd_vel", RosGeometryMsgsTwist)``.
* Factory methods such as ``create_request()``, ``create_goal()`` or ``create_result()`` are declared to return ``RosMsg``. The object really is the typed class; annotate the variable to get autocompletion:

  .. code-block:: gdscript

      var req: RosExampleInterfacesSrvAddTwoIntsRequest = client.create_request()

* A field whose name clashes with a :ref:`RosMsg<class_RosMsg>` member or a GDScript keyword is not exposed as a property; read and write it with ``get_member()`` / ``set_member()``. Run Godot with ``--verbose`` to see which fields were left out.

Dynamic access
--------------

Typed classes are a convenience, not a requirement. Any type, including one that is not among your dependencies, can be created with ``RosMsg.from_type()`` and accessed the same way, just without editor typing:

.. code-block:: gdscript

    var msg = RosMsg.from_type("my_msgs/msg/RobotStatus")
    msg.name = "Titan-1"
    msg.position.x = randf_range(-10.0, 10.0)
    pub.publish(msg)

``from_type()`` returns the typed class when the type has one, and ``null`` if the type can't be resolved (for example, the package isn't sourced).

Exported games
--------------

rclgd reads ``package.xml`` at startup in exported games too, but Godot only exports resources by default. Add ``package.xml`` to the export preset's **Resources > Filters to export non-resource files/folders**; otherwise the exported game has no typed classes and scripts using them fail to load.

Upgrading from generated wrappers
---------------------------------

Earlier rclgd versions generated GDScript wrappers into ``res://addons/rclgd/gen`` from an editor plugin. Both are gone. In existing projects:

* delete ``addons/rclgd/gen``: the native classes replace it with the same names, and its ``class_name`` scripts clash with them;
* delete ``addons/rclgd/plugin.cfg`` and ``addons/rclgd/rclgd.gd``, and remove the plugin from ``[editor_plugins]`` in ``project.godot`` (it calls the removed ``RosMsg.gen_editor_support()``). The ``ROS`` autoload stays.

``ros2 rclgd doctor`` flags both leftovers.
