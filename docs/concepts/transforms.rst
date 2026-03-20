Transforms & Coordinate Systems
===============================

Handling 3D coordinate systems correctly is vital for seamless integration between Godot Engine and ROS 2. This page explains the differences and how **rclgd** bridges them.

Coordinate System Comparison
----------------------------

Both Godot and ROS 2 use a **Right-Handed** coordinate system, but they differ in orientation.

.. list-table:: System Axis Differences
   :widths: 30 35 35
   :header-rows: 1

   * - Feature
     - Godot Engine
     - ROS 2 (REP-103)
   * - **Coordinate System**
     - Right-Handed
     - Right-Handed
   * - **Up Axis**
     - +Y (Green)
     - +Z (Blue)
   * - **Forward Axis**
     - -Z (Standard) / +Z (rclgd)
     - +X (Red)
   * - **Right Axis**
     - +X (Red)
     - -Y (Green)

Visual Representation
---------------------

To clarify the mapping, consider how Godot's axes relate to ROS 2's REP-103:

.. image:: ../img/gdaxis.svg
   :align: center
   :alt: Godot to ROS 2 Axis Mapping Diagram

Axis Mapping in rclgd
---------------------

To simplify integration, **rclgd** uses a canonical mapping where Godot's **+Z** is treated as Forward. This allows for a clean transition to ROS 2's REP-103 standard.

.. list-table:: Canonical Axis Reference
   :widths: 30 30 40
   :header-rows: 1

   * - Godot Direction (+Z Forward)
     - ROS 2 Direction
     - Message Value mapping
   * - **+Z (Forward)**
     - **+X (Forward)**
     - ``t.origin.z`` -> ``translation.x``
   * - **+X (Right)**
     - **+Y (Left)**
     - ``t.origin.x`` -> ``translation.y``
   * - **+Y (Up)**
     - **+Z (Up)**
     - ``t.origin.y`` -> ``translation.z``

GDScript Examples
-----------------

Manual Transform Conversion
~~~~~~~~~~~~~~~~~~~~~~~~~~~

The following example shows how to manually map a Godot ``Transform3D`` to a ROS 2 ``geometry_msgs/msg/Transform``.

.. code-block:: gdscript

    func godot_to_ros_transform(t: Transform3D) -> RosMsg:
        var msg = RosMsg.from_type("geometry_msgs/msg/Transform")
        
        # 1. Map Position (Origin)
        msg.translation.x = t.origin.z
        msg.translation.y = t.origin.x
        msg.translation.z = t.origin.y
        
        # 2. Map Rotation (Quaternion)
        # We need to rotate the basis to match the target frame
        var ros_basis = Basis()
        var g_forward = t.basis.z
        var g_right = t.basis.x
        var g_up = t.basis.y
        
        ros_basis.x = Vector3(g_forward.z, g_forward.x, g_forward.y)
        ros_basis.y = Vector3(g_right.z, g_right.x, g_right.y)
        ros_basis.z = Vector3(g_up.z, g_up.x, g_up.y)
        
        var q = ros_basis.get_quaternion()
        msg.rotation.x = q.x
        msg.rotation.y = q.y
        msg.rotation.z = q.z
        msg.rotation.w = q.w
        
        return msg

Using TF Broadcasters and Listeners
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The `RosTfBroadcaster` and `RosTfListener` classes provide high-level abstractions for managing transforms.

.. code-block:: gdscript

    var broadcaster: RosTfBroadcaster
    var listener: RosTfListener

    func setup_tf_demo():
        # Create from the node
        broadcaster = ros_node.create_tf_broadcaster()
        listener = ros_node.create_tf_listener()

    func publish_my_move(t: Transform3D):
        # send_transform(transform, frame_id, parent_frame_id, is_static)
        broadcaster.send_transform(t, "base_link", "odom")

    func get_robot_position():
        # returns Transform3D automatically converted for Godot
        var t = listener.lookup_transform("odom", "base_link")
        if t:
            print("Robot Pose: ", t.origin)
