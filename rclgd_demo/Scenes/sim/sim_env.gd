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
	demo_pub = ros_node.create_publisher("/gd_topic", "std_msgs/msg/String")
	demo_sub = ros_node.create_subscriber("/gd_topic", "std_msgs/msg/String", _on_status_received)
	
	#RosMsg.gen_editor_support("sensor_msgs/msg/PointCloud2","res://addons/rclgd/gen")
	# 4. Start a periodic timer to publish
	get_tree().create_timer(1.0).timeout.connect(publish_test_msg)
	

func publish_test_msg():
	"""
	Message Types are instantiated by the from_type static method,
	once created you can access their fields as you would normally do in any 
	other rcl implementation.
	"""
	var msg = RosMsg.from_type("std_msgs/String")
	msg.data = "Hi there from Godot!"
	demo_pub.publish(msg)

# Callbacks from subscriber are triggered on message
func _on_status_received(msg: RosMsg):
	print(msg)
