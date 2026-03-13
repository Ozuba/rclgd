Introduction to rclgd
=====================

**rclgd** (Record Linkage for Godot - actually ROS Client Library for Godot) provides a high-performance ROS2 integration for Godot Engine 4.

Why rclgd?
----------

Traditional ROS-Godot integrations often rely on WebSockets or bridge nodes, which can introduce latency and complexity. `rclgd` runs natively as a GDExtension, allowing for:

1. **Direct Communication**: Shared memory and local transport via DDS.
2. **Minimal Latency**: No intermediate bridge processing.
3. **Full ROS2 Power**: Access to parameters, services, and complex type support.

Core Concepts
-------------

- **Nodes**: The basic unit of communication in ROS2.
- **Messages**: Data structures passed between nodes.
- **Topics**: Named channels for publishing/subscribing.
- **Services**: Request/Response communication pattern.
- **Parameters**: Dynamic configuration for nodes.

`rclgd` maps these concepts directly into Godot objects that feel natural to GDScript developers.
