Parameter Management
====================

Parameters allow you to configure ROS 2 nodes dynamically. **rclgd** provides a seamless interface for declaring, updating, and monitoring parameters from GDScript.

Working with Parameters
-----------------------

Parameters must be declared before use. You can then get or set their values using standard Godot types.

.. code-block:: gdscript

    func manage_params():
        # Declare with default value
        ros_node.declare_parameter("max_speed", 2.0)
        
        # Get and Set
        var speed = ros_node.get_parameter("max_speed")
        ros_node.set_parameter("max_speed", speed * 1.5)

Dynamic Updates via Signals
---------------------------

The `parameter_changed` signal allows your node to react to parameter updates from external ROS 2 tools (e.g., `ros2 param set`).

.. code-block:: gdscript

    func _init():
        ros_node.parameter_changed.connect(_on_param_changed)

    func _on_param_changed(name: String, value: Variant):
        print("Parameter %s updated: %s" % [name, str(value)])

Type Mapping Reference
----------------------

**rclgd** automatically converts between Godot ``Variant`` and ROS 2 parameter types.

.. list-table:: Supported Types
   :widths: 40 60
   :header-rows: 1

   * - Godot Type
     - ROS 2 Parameter Type
   * - ``bool``
     - ``PARAMETER_BOOL``
   * - ``int``
     - ``PARAMETER_INTEGER``
   * - ``float``
     - ``PARAMETER_DOUBLE``
   * - ``String``
     - ``PARAMETER_STRING``
   * - ``PackedByteArray``
     - ``PARAMETER_BYTE_ARRAY``
   * - ``PackedInt32Array`` / ``PackedInt64Array``
     - ``PARAMETER_INTEGER_ARRAY``
   * - ``PackedFloat32Array`` / ``PackedFloat64Array``
     - ``PARAMETER_DOUBLE_ARRAY``
   * - ``PackedStringArray``
     - ``PARAMETER_STRING_ARRAY``
