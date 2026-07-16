extends Node

var ros_node: RosNode
var demo_pub: RosPublisher
var demo_sub: RosSubscriber
var demo_timer: RosTimer  # keep a reference — dropping it cancels the timer


func _ready() -> void:
	# The ROS autoload (res://addons/rclgd/ros_init.gd) already called
	# rclgd.init() with the command line --ros-args before any scene loaded.
	ros_node = RosNode.new()
	ros_node.init("$name")

	demo_pub = ros_node.create_publisher("/$name/chatter", "std_msgs/msg/String")
	demo_sub = ros_node.create_subscription("/$name/chatter", "std_msgs/msg/String", _on_msg)
	demo_timer = ros_node.create_timer(1.0, _publish)


func _publish() -> void:
	var msg = RosMsg.from_type("std_msgs/msg/String")
	msg.data = "Hello from $name!"
	demo_pub.publish(msg)


func _on_msg(msg: RosMsg) -> void:
	print("[$name] heard: ", msg.data)
