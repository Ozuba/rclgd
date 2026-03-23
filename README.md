<div align="center">
	
# RCLGD

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.18440938.svg)](https://doi.org/10.5281/zenodo.18440938)

<img src="docs/img/icon.svg" align="center" width="400" alt="RCLGD  logo">

An implementation of a ros2 client library for the Godot Engine
based on [ROS Babel Fish](https://github.com/LOEWE-emergenCITY/ros_babel_fish)


</div>

## Features
As for now only the basic set of the rclcpp api are implemented, keep in mind this is highly experimental and not suited yet for production. But it serves as the groundbase for implementing awesome simulation enviroments and visualizers using the powerful features of the godot engine.

- [x] rclgd Singleton
- [x] Dynamic Msg Type Support
- [x] Publishers
- [x] Subscribers
- [x] TF2 Listeners And Broadcasters as 3D Nodes in godot
- [x] Service Clients
- [x] Service Servers
- [x] Timers
- [ ] Actions
- [x] Parameters
- [x] Node and TF Namespacing
- [x] ROS Graph Inspection
- [x] QoS -> Through QoS RosQoS resource
- [x] Godot template project
- [x] Godot Editor Support -> Pseudo-Static Type Wrappers
- [x] Simulation Time -> Use `--ros-args -p use_sim_time:=true` 
- [x] Native RCLGD packages in colcon



## Installation
Clone this package into your workspace and install dependencies, build it and source it.

```bash
rosdep install --from-paths src --ignore-src -y -r
colcon build --packages-select rclgd
source install/install.sh
```

> [!NOTE]
> rclgd has been tested with Godot 4.5 & 4.6.  
> You can change the target by retargetting the submodule to corresponding tag and using `-DGODOT_VERSION=4.5`


## Integration
This package is intented to work as a support package, once you build it you can create rclgd packages based on this
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
In order to edit those packages you will need to launch the godot editor by `ros2 run rclgd godot` this will ensure that `librclgd.so`
is accesible by all projects containing the corresponding `rclgd.gdextension`

Once you run `colcon build` and source your installation you will be able to run your godot-ros applcation
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

	# 3. Setup Publisher & Subscriber
	demo_pub = ros_node.create_publisher("/gd_topic", "std_msgs/msg/String.msg")
	demo_sub = ros_node.create_subscriber("/gd_topic", "std_msgs/msg/String.msg", _on_status_received)

	# 4. Start a periodic timer to publish
	get_tree().create_timer(1.0).timeout.connect(publish_test_msg)
	

func publish_test_msg():
	"""
	Message Types are instantiated by the from_type static method,
	once created you can access their fields as you would normally do in any 
	other rcl implementation.
	"""
	var msg = RosMsg.from_type("std_msgs/msg/String.msg")
	msg.data = "Hi there from Godot!"
	demo_pub.publish(msg)

# Callbacks from subscriber are triggered on message
func _on_status_received(msg: RosMsg):
	print(msg)

```

## A note on performance
The type masking system used by rclgd depends at this moment in transfering data between godot and ros contexts, however preliminary tests show that working with high bandwidth types like `PointCloud2` with over 250.000 points its handled nicely.

## Disclaimer

> [!NOTE]
> This is for now a demonstration project and not suited for production, things arent polished and are expected to break, feel free to open any 
> issues you find out.
