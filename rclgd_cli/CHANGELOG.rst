^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rclgd_cli
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Breaking: the project template no longer ships the editor plugin
  (addons/rclgd/plugin.cfg and rclgd.gd) that generated typed message
  wrappers; message types are now native classes registered by rclgd
  itself. The ROS autoload stays, declared directly in project.godot.
  Existing projects should delete those two files and addons/rclgd/gen,
  and drop the plugin from [editor_plugins]
* Feature: ros2 rclgd doctor warns about leftovers of the generated
  wrappers: an addons/rclgd/gen folder or the old editor plugin

3.0.0 (2026-10-22)
------------------
* Fix: template rclgd.gdextension now declares compatibility_minimum
  4.7, matching the Godot API rclgd is built against (was 4.3)

2.1.0 (2026-07-17)
------------------
* Initial release: ros2 rclgd command with create, editor, list and
  doctor verbs, plus the editor addon template
* Engine provisioning moved into the rclgd build; the former setup verb
  was removed
* Contributors: Miguel Oroz Zubasti
