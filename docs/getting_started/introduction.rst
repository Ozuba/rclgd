Introduction to rclgd
=====================

**rclgd** (ROS Client Library for Godot) integrates ROS 2 natively into Godot Engine 4.

Why rclgd?
----------

Many ROS-Godot integrations go through a bridge: WebSockets with ``rosbridge``,
or a separate bridge node. Every message is then serialized twice (once to JSON
or a custom format, once to DDS) and crosses an extra process.

**rclgd** runs as a GDExtension and exposes ``rclcpp`` directly to GDScript, so
your Godot project *is* a ROS 2 node:

1. **Same transport as any C++ node**: messages go straight to DDS, including
   intra-host shared memory where the middleware supports it.
2. **No bridge process**: nothing to launch, configure or keep in sync.
3. **The whole ROS 2 feature set**: parameters, QoS, services, actions, TF2,
   simulation time and logging, all from GDScript.

Core Concepts Mapping
---------------------

ROS 2 concepts map to Godot objects:

*   **Nodes** (``RosNode``): the unit of communication on the ROS 2 graph.
*   **Messages** (``RosMsg``): any ROS 2 message, resolved at runtime by
    BabelFish. Interfaces your ``package.xml`` depends on also get native typed
    classes such as ``RosGeometryMsgsTwist`` (see :doc:`/tutorials/custom_messages`).
*   **Topics** (``RosPublisher`` / ``RosSubscriber``): continuous data streams,
    with optional ``RosQoS`` profiles.
*   **Services** (``RosService`` / ``RosClient``): request/response, awaitable
    from GDScript.
*   **Actions** (``RosActionServer`` / ``RosActionClient``): long-running,
    cancelable goals with feedback.
*   **Timers** (``RosTimer``): periodic callbacks driven by the node's clock,
    including simulation time.
*   **Parameters**: declared on a ``RosNode``, with a signal for external changes.
*   **Logging** (``RosLogger``): rclpy-style logging to the console and ``/rosout``.
*   **Transforms** (``RosTfBroadcaster`` / ``RosTfListener``): TF2 with automatic
    conversion between the Godot and ROS axis conventions.
