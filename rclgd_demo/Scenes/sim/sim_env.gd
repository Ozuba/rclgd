extends Node

var ros_node: RosNode
var car_pub: RosPublisher
var car_sub: RosSubscriber

func _ready() -> void:
	# 1. Initialize Global ROS Context
	if not rclgd.ok():
		rclgd.init([])

	# 2. Create the Standalone Node (RefCounted)
	ros_node = RosNode.new()
	ros_node.init("godot_controller_node")

	# 3. Setup Publisher & Subscriber
	car_pub = ros_node.create_publisher("/car_cmd", "mirena_common/msg/Car")
	car_sub = ros_node.create_subscriber("/car_cmd", "mirena_common/msg/Car", _on_status_received)

	# 4. Start a periodic timer to publish
	get_tree().create_timer(1.0).timeout.connect(publish_test_msg)
	

func publish_test_msg():
	var msg = RosMsg.from_type("mirena_common/msg/Car")
	msg.x = 1.5
	msg.y = 0.0
	
	car_pub.publish(msg)
	print("Published command: x=1.5")

# This runs safely on the Main Thread
func _on_status_received(msg: RosMsg):
	print(msg)
