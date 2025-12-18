# RCLGD
An implementation of a ros2 client library for the Godot Engine
based on [ROS Babel Fish](https://github.com/LOEWE-emergenCITY/ros_babel_fish)

## Features
As for now only the basic set of the rclcpp api are implemented, keep in mind this is highly experimental and not suited yet for production. But it serves as the groundbase for implementing awesome simulation enviroments and visualizers using the powerful features of the godot engine.

- [x] rclgd Singleton
- [x] Dynamic Msg Type Support
- [x] Basic Publishers
- [x] Basic Subscribers
- [ ] TF2 Helpers (Make transform integrate Nicely with godot transform system)
- [ ] Service Clients
- [ ] Service Servers
- [ ] Actions
- [ ] QoS


## Demo Usage 
The Project is structured as a ros2 package and includes all neccesary things for automated building of the extension and launching either the demo `ros2 run rclgd rglcd_demo` or the 
godot editor `ros2 run rclgd godot`, in order to start testing
the implementation.

## Integration
rclgd is a self contained gdextension integrating rclgd in your own godot project is as simple as dropping `librclgd.so` and the corresponding `rclgd.gdextension` into you godot file tree.

> [!IMPORTANT]
> You need to have a workspace sourced in your execution environment in
> order for rclgd to be able to work. Not doing so will probably result in a crash by now.

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
		rclgd.init([])

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
