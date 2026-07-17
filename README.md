<div align="center">
	
# RCLGD

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.18440938.svg)](https://doi.org/10.5281/zenodo.18440938)

<img src="docs/img/icon.svg" align="center" width="400" alt="RCLGD  logo">

An implementation of a ros2 client library for the Godot Engine
based on [ROS Babel Fish](https://github.com/LOEWE-emergenCITY/ros_babel_fish)


</div>

## Features
As for now only the basic set of the rclcpp api are implemented, keep in mind this is highly experimental and not suited yet for production. But it serves as the groundbase for implementing awesome simulation environments and visualizers using the powerful features of the godot engine.

- [x] rclgd Singleton
- [x] Dynamic Msg Type Support
- [x] Nodes
- [x] Publishers
- [x] Subscribers
- [x] Service Clients
- [x] Service Servers
- [x] Timers
- [x] Action Clients & Servers
- [x] Parameters
- [x] TF2 Publishers and Listeners
- [x] TF2 Listeners And Broadcasters as 3D Nodes in godot (Will be deprecated in future update)
- [x] Node and TF Namespacing
- [x] ROS Graph Inspection
- [x] QoS -> Through QoS RosQoS resource
- [x] Godot template project
- [x] Godot Editor Support -> Pseudo-Static Type Wrappers
- [x] Simulation Time -> `-p publish_sim_time:=true` publishes the Godot physics clock on `/clock`; the standard `-p use_sim_time:=true` makes clocks and timers follow `/clock`
- [x] Native RCLGD packages in colcon



## Repository layout
The suite is split into independently releasable ROS 2 packages:

| Package | Type | Contents |
|---|---|---|
| `rclgd` | ament_cmake | the GDExtension (librclgd.so), godot launcher, runtime manifest |
| `colcon_rclgd` | ament_python | the `build_type: rclgd` colcon extension |
| `rclgd_cli` | ament_python | the `ros2 rclgd` command and the editor addon template |

The system-test suite (`rclgd_tests`, itself a native rclgd package) is
maintained separately so this repository contains only the releasable
packages.

## Installation
Clone this repository into your workspace and install dependencies, build it and source it.
The build downloads the pinned Godot editor binary (SHA-512 verified against
the official release sums) and installs it as `lib/rclgd/godot-bin`, next to
`librclgd.so` — the install is complete out of the box, no extra step.

```bash
rosdep install --from-paths src --ignore-src -y -r
colcon build --packages-up-to rclgd
source install/setup.bash
```

To run the system tests, put the `rclgd_tests` package in your workspace and
build it (it dogfoods the whole `build_type: rclgd` pipeline — it needs the
suite above built and sourced first), then run it like any other rclgd
package:

```bash
colcon build --packages-select rclgd_tests && source install/setup.bash
ros2 run rclgd_tests rclgd_tests --headless   # exits 0 only if all tests pass
```

The build installs exactly the Godot version librclgd was compiled against
(the version of the godot-cpp bindings; recorded in
`share/rclgd/godot_version`). To use a different Godot version, retarget the
`godot-cpp` submodule to the corresponding branch, edit `GODOT_VERSION` and
the `GODOT_SHA512_*` pins in `rclgd/CMakeLists.txt`, and rebuild.

## Command line tools
rclgd ships a `ros2 rclgd` command:

| Verb | Purpose |
|---|---|
| `ros2 rclgd create <name>` | Scaffold a new rclgd package (package.xml, project, addon, demo pub/sub) |
| `ros2 rclgd editor [pkg]` | Open the Godot editor on a package's **source** project (no argument: current directory, or the project selection screen) |
| `ros2 rclgd list` | List built rclgd packages |
| `ros2 rclgd doctor` | Diagnose broken setups (versions, extension wiring, imports) |

Typed GDScript wrappers (shadow classes) are generated automatically: when a
project opens in the editor, the rclgd plugin reads the dependencies declared
in the project's `package.xml` and (re)generates wrappers for their message
types into `res://addons/rclgd/gen`. Add a `<depend>` to package.xml, reopen
the editor (or use *Project > Tools > Regenerate ROS2 Types*), and the typed
classes appear. The generated wrappers are plain GDScript files meant to be
committed with the project — they regenerate automatically whenever the
dependency set changes.


## Integration
This package is intended to work as a support package, once you build it you can create rclgd packages based on this
[Template](https://github.com/Ozuba/rclgd-template) in your favourite ros workspace.

```
rclgd_ws
└── src
    ├── rclgd
    ├── rclgd-template
    └── rclgd_demo
```
A typical rclgd `package.xml` looks like
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>rclgd-template</name>
  <version>0.1.0</version>
  <maintainer email="example@example.com">Ozuba</maintainer>
  <license>MIT</license>

  <buildtool_depend>rclgd</buildtool_depend>
  <export>
	<build_type>rclgd</build_type>
  </export>
</package>
```
In order to edit those packages launch the godot editor through
`ros2 rclgd editor <package>` (plain `ros2 rclgd editor` opens the project
selection screen; `ros2 run rclgd godot` runs the raw engine binary) — this
resolves the right Godot binary and ensures `librclgd.so` is accessible by
all projects containing the corresponding `rclgd.gdextension`

Once you run `colcon build` and source your installation you will be able to run your godot-ros application
as any other ros executable.
> [!TIP]
> Example: `ros2 run rclgd_demo rclgd_demo`


## Usage
Attach a script to you favourite node and start publishing and subscribing things!


```GDscript
extends Node

var ros_node: RosNode
var demo_pub: RosPublisher
var demo_sub: RosSubscriber

func _ready() -> void:
	# 1. Initialize Global ROS Context
	"""
	The rclgd singleton manages the rclcpp Context
	and should be started by the user, in rclgd for now, theres no need to handle
	node spinning as it is done in a background thread safely in order to avoid blocking
	the godot main thread.
	"""
	if not rclgd.ok():
		rclgd.init()

	# 2. Create the Standalone Node (RefCounted)
	ros_node = RosNode.new()
	ros_node.init("godot_controller_node")

	# 3. Setup Publisher & Subscription
	demo_pub = ros_node.create_publisher("/gd_topic", "std_msgs/msg/String")
	demo_sub = ros_node.create_subscription("/gd_topic", "std_msgs/msg/String", _on_status_received)

	# 4. Start a periodic timer to publish
	get_tree().create_timer(1.0).timeout.connect(publish_test_msg)
	

func publish_test_msg():
	"""
	Message Types are instantiated by the from_type static method,
	once created you can access their fields as you would normally do in any 
	other rcl implementation.
	"""
	var msg = RosMsg.from_type("std_msgs/msg/String")
	msg.data = "Hi there from Godot!"
	demo_pub.publish(msg)

# Callbacks from subscriptions are triggered on message
func _on_status_received(msg: RosMsg):
	print(msg)

```

### Actions
Action servers auto-accept incoming goals and hand them to your execute callback as a [RosServerGoalHandle]; drive each goal to exactly one terminal state (`succeed`, `abort` or `canceled`). Clients get a [RosGoalHandle] back from `send_goal`, which exposes `feedback` and `completed` signals you can `await`.

```GDScript
# Server: classic Fibonacci action
var server = ros_node.create_action_server("fibonacci", "example_interfaces/action/Fibonacci",
	func(goal_handle):
		var sequence = [0, 1]
		for i in range(2, goal_handle.get_goal().order):
			if goal_handle.is_cancel_requested():
				var canceled_result = goal_handle.create_result()
				canceled_result.sequence = sequence
				goal_handle.canceled(canceled_result)
				return
			sequence.append(sequence[i - 1] + sequence[i - 2])
			var fb = goal_handle.create_feedback()
			fb.sequence = sequence
			goal_handle.publish_feedback(fb)
		var result = goal_handle.create_result()
		result.sequence = sequence
		goal_handle.succeed(result)
)

# Client
var client = ros_node.create_action_client("fibonacci", "example_interfaces/action/Fibonacci")
if client.wait_for_server(2.0):
	var goal = client.create_goal()
	goal.order = 10
	var goal_handle = client.send_goal(goal)
	goal_handle.feedback.connect(func(msg): print("Partial: ", msg.sequence))
	await goal_handle.completed
	if goal_handle.get_status() == RosGoalHandle.STATUS_SUCCEEDED:
		print("Result: ", goal_handle.get_result().sequence)
```
A running goal can be canceled from the client with `goal_handle.cancel()`; the server sees it through `is_cancel_requested()` and the final status arrives via the `completed` signal as `STATUS_CANCELED`.

## Coordinate conventions
Godot and ROS are both right-handed but use different axis conventions. RCLGD maps between them with a fixed permutation used consistently by the TF broadcaster and listener:

| ROS              | Godot            |
|------------------|------------------|
| +X (forward)     | -Z (forward)     |
| +Y (left)        | -X (left)        |
| +Z (up)          | +Y (up)          |

If you build poses, twists or other geometry by hand, use the same mapping through the helpers exposed on the singleton: `rclgd.godot_to_ros_vector()`, `rclgd.ros_to_godot_vector()`, `rclgd.godot_to_ros_quat()` and `rclgd.ros_to_godot_quat()`.

## A note on threading
By default the ROS executor spins in a separate thread (`use_separate_thread:=true`). Subscription and timer callbacks are deferred to the Godot main thread, so regular GDScript code is safe. **Service server callbacks are the exception**: they run synchronously on the executor thread (the response must be filled before returning to the client), so keep them self-contained and use `call_deferred` for any side effects on the scene. With `--ros-args -p use_separate_thread:=false` the executor spins on the physics tick and everything runs on the main thread.

## A note on performance
The type masking system used by rclgd depends at this moment in transfering data between godot and ros contexts, however preliminary tests show that working with high bandwidth types like `PointCloud2` with over 250.000 points its handled nicely.

## Disclaimer

> [!NOTE]
> This is for now a demonstration project and not suited for production, things arent polished and are expected to break, feel free to open any 
> issues you find out.
