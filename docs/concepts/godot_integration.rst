Godot Integration
=================

Integrating **rclgd** into your Godot project follows standard GDExtension patterns.

Initialization
--------------

You typically initialize ROS2 at the start of your application.

.. code-block:: gdscript

    func _init():
        # rclcpp initialization happens automatically in the GDExtension entry point
        pass

Using RefCounted
----------------

Most `rclgd` classes inherit from `RefCounted`. This means you don't need to manually free them; they will be cleaned up when no longer referenced.

.. code-block:: gdscript

    var node: RosNode
    
    func create_temp_node():
        var tmp = RosNode.new()
        tmp.init("temp")
        # tmp will be freed when this function returns unless stored elsewhere

Signals vs Callbacks
--------------------

While Godot heavily uses signals, `rclgd` currently uses `Callable` callbacks for high-frequency data (like subscriptions) to minimize overhead. You can easily emit signals from these callbacks if needed.

.. code-block:: gdscript

    signal message_received(data)

    func _on_ros_message(msg: RosMsg):
        emit_signal("message_received", msg.get_member("data"))
