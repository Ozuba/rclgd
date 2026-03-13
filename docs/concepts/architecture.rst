Architecture
============

**rclgd** is built with a focus on flexibility and performance, bridging the gap between Godot Engine's GDExtension system and ROS2.

BabelFish Integration
---------------------

A core component of `rclgd` is `ros_babel_fish`. This library allows `rclgd` to handle ROS2 messages dynamically at runtime.

- **No Recompilation**: You don't need to recompile your GDExtension every time you add a new custom ROS2 message.
- **Type Mapping**: `rclgd` automatically maps ROS2 primitive types to Godot `Variant` types.
- **Complex Types**: Nested messages and arrays are handled recursively.

rclcpp and Threading
--------------------

`rclgd` uses the standard `rclcpp` executor in the background. 

- **Thread Safety**: Communication between ROS2 threads and the Godot main thread is handled safely.
- **Node Lifecycle**: `RosNode` instances manage the underlying `rclcpp::Node` lifetime.

GDExtension Wrapper
-------------------

The classes exposed to GDScript (like `RosNode`) are wrappers around the C++ implementation. This adds minimal overhead while providing a clean, Godot-like API.
