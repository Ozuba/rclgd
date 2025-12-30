extends Node

var ros_node: RosNode
var demo_pub: RosPublisher
var demo_sub: RosSubscriber
var inplace_pub :RosPublisher
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
	inplace_pub = ros_node.create_publisher("/test", "geometry_msgs/PoseArray")

	#RosMsg.gen_editor_support("sensor_msgs/msg/PointCloud2","res://addons/rclgd/gen")
	# 4. Start a periodic timer to publish
	test_object_array_edit_in_place()

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


func test_object_array_edit_in_place():
	# 1. Create a PoseArray (contains an array of Compound objects)
	var msg = RosMsg.from_type("geometry_msgs/PoseArray")
	
	# 2. Initialize array with 2 empty poses
	# Calling set_member/property trigger's the ROS buffer resize
	var poses = [RosGeometryMsgsPose.new(), RosGeometryMsgsPose.new()] 
	
	# 4. MODIFY IN PLACE
	poses[0].position.x = 123.45
	msg.poses.append(poses[0])
	inplace_pub.publish(msg)
