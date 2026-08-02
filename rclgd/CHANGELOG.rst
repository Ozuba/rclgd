^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rclgd
^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.1 (2026-08-02)
------------------
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
