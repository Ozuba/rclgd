^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rclgd
^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
