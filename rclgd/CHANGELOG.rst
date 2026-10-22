^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rclgd
^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.0 (2026-10-22)
------------------
* Feature: Added RosLogger with rclpy logger API parity: debug/info/
  warning/error/fatal with once, skip_first and throttle_duration_sec
  filters, get_child, set_level and get_effective_level. Get one from
  RosNode.get_logger() (console and /rosout) or rclgd.get_logger(name)
  (console only); rclgd also gains set_logger_level,
  get_logger_effective_level and get_logging_severity_from_string
* Change: classes created by RosNode / rclgd factories (publishers,
  subscribers, clients, services, requests, timers, action clients and
  servers, goal handles, TF broadcaster and listener, RosLogger) are now
  abstract, so ``.new()`` fails instead of returning an unusable object.
  RosNode, RosMsg and RosQoS remain instantiable
* Bugfix: RosGoalHandle.STATUS_CANCELED and STATUS_ABORTED had swapped
  values (6 and 5), so canceled goals reported STATUS_ABORTED and aborted
  goals STATUS_CANCELED. They now match action_msgs/GoalStatus (5 and 6)
* Bugfix: freeing a RosTfListener while a /tf message was being handled
  could corrupt the heap and crash or hang the process; the listener now
  owns its /tf subscriptions and keeps the buffer alive for callbacks
  still in flight
* Bugfix: RosService request callbacks and RosNode parameter callbacks
  no longer reach the object through a raw pointer, so freeing a service
  or node while one of its callbacks runs is safe
* Testing: headless GDScript system test suite, run by ``colcon test``
* Bugfix: fixed liveliness unavailability inside RosQoS
* Feature: Added support for stamped TF lookups and other TF listener features
* Bugfix: TF lookup timeouts are now honoured; the buffer was never told the
  executor runs on its own thread, so any non-zero timeout was rejected
* Contributors: Miguel Oroz Zubasti

2.1.0 (2026-07-17)
------------------
* Restructure into a three-package suite: rclgd (GDExtension),
  colcon_rclgd (build type) and rclgd_cli (ros2 CLI)
* Download the Godot engine at build time (SHA-512 pinned per
  architecture) and install it as lib/rclgd/godot-bin, making the
  package self-contained
* Upgrade godot-cpp and share libstdc++ across the extension boundary
* Lazy initialization architecture and runtime hardening
* Proper memory management for the shadow script registry
* Contributors: Miguel Oroz Zubasti
