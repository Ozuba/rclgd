Introduction to rclgd
=====================

**rclgd** (ROS Client Library for Godot) provides a high-performance, native ROS 2 integration for Godot Engine 4.

Why rclgd?
----------

Traditional ROS-Godot integrations often rely on WebSockets (like `rosbridge`) or intermediate standalone bridge nodes. These approaches introduce network latency, serialization overhead, and significant architectural complexity. 

**rclgd** solves this by running natively as a GDExtension, directly exposing the official `rclcpp` API to GDScript. This allows for:

1. **Direct Communication**: Utilizes shared memory and local transport via DDS, exactly like a standard C++ ROS 2 node.
2. **Zero-Latency Bridging**: No intermediate JSON serialization or bridge processes.
3. **Full ROS 2 Power**: Gain direct access to Parameters, Services, Timers, and complex Message types right from the Godot Editor.

Core Concepts Mapping
---------------------

**rclgd** maps standard ROS 2 concepts directly into Godot objects that feel natural and idiomatic to GDScript developers:

*   **Nodes** (`RosNode`): The basic unit of communication, handling connections to the ROS 2 graph.
*   **Messages** (`RosMsg`): Dynamic data structures passed between nodes, powered by BabelFish.
*   **Topics** (`RosPublisher` / `RosSubscriber`): Named channels for continuous data streams.
*   **Services** (`RosService` / `RosClient`): Asynchronous Request/Response communication patterns.
*   **Timers** (`RosTimer`): Executor-synchronized periodic tasks.

By bringing these concepts natively into GDScript, **rclgd** empowers you to build complex robotics simulations and visualizations with exceptional performance.
