Parameter Management
====================

Parameters configure ROS 2 nodes at launch and at runtime. **rclgd** lets you declare, read, update and monitor them from GDScript with plain Godot types.

Working with Parameters
-----------------------

Parameters must be declared before use. You can then get or set their values:

.. code-block:: gdscript

    func _ready():
        ros_node = RosNode.new()
        ros_node.init("my_godot_node")

        # Declare with a default value
        ros_node.declare_parameter("max_speed", 2.0)

        # Get and set
        var speed = ros_node.get_parameter("max_speed")
        ros_node.set_parameter("max_speed", speed * 1.5)

A value given at launch replaces the declared default, exactly as for any ROS node. It can come from the command line (``--ros-args -p max_speed:=3.0``) or, in projects created with ``ros2 rclgd create``, from a project setting named ``ros/parameters/max_speed`` (see :doc:`/tutorials/basic_usage`).

Reacting to Updates
-------------------

The ``parameter_changed`` signal lets your node react to updates from external tools (e.g. ``ros2 param set``). Connect it after ``init()``:

.. code-block:: gdscript

    func _ready():
        # ... after ros_node.init(...)
        ros_node.parameter_changed.connect(_on_param_changed)

    func _on_param_changed(name: String, value: Variant):
        print("Parameter %s updated: %s" % [name, str(value)])

Type Mapping Reference
----------------------

**rclgd** converts between Godot ``Variant`` and ROS 2 parameter types:

.. list-table:: Supported Types
   :widths: 45 30 25
   :header-rows: 1

   * - Godot type (when setting)
     - ROS 2 parameter type
     - Godot type (when reading)
   * - ``bool``
     - ``PARAMETER_BOOL``
     - ``bool``
   * - ``int``
     - ``PARAMETER_INTEGER``
     - ``int``
   * - ``float``
     - ``PARAMETER_DOUBLE``
     - ``float``
   * - ``String``
     - ``PARAMETER_STRING``
     - ``String``
   * - ``PackedByteArray``
     - ``PARAMETER_BYTE_ARRAY``
     - ``PackedByteArray``
   * - ``PackedInt32Array`` / ``PackedInt64Array``
     - ``PARAMETER_INTEGER_ARRAY``
     - ``PackedInt64Array``
   * - ``PackedFloat32Array`` / ``PackedFloat64Array``
     - ``PARAMETER_DOUBLE_ARRAY``
     - ``PackedFloat64Array``
   * - ``PackedStringArray``
     - ``PARAMETER_STRING_ARRAY``
     - ``PackedStringArray``
   * - ``Array`` of ``bool``
     - ``PARAMETER_BOOL_ARRAY``
     - ``Array``

A plain ``Array`` takes its parameter type from its first element (``bool``, ``int``, ``float`` or ``String``); an empty ``Array`` has no type to infer and is passed on as an unset value, so prefer a ``Packed*Array``.
