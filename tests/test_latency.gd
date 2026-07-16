extends Test

signal message_looped_back

var ros_node: RosNode
var pub: RosPublisher
var sub: RosSubscriber

var start_time: float = 0.0
var rtt_ms: float = 0.0

func before_test():
	if not rclgd.ok(): rclgd.init([])
	
	ros_node = RosNode.new()
	ros_node.init("latency_self_test", "")
	
	var topic = "/rclgd/loopback_test"
	pub = ros_node.create_publisher(topic, "std_msgs/msg/String")
	sub = ros_node.create_subscription(topic, "std_msgs/msg/String", _on_message)
	
	# Wait for self-discovery
	await get_tree().create_timer(1.0).timeout

func _execute():
	var msg = RosMsg.from_type("std_msgs/msg/String")
	msg.data = "LATENCY_CHECk"

	start_time = Time.get_ticks_usec()
	pub.publish(msg)

	# Await the signal from our own subscriber with a timeout
	var timeout = 2.0
	var current_time = 0.0
	while rtt_ms == 0.0 and current_time < timeout:
		await get_tree().create_timer(0.01).timeout
		current_time += 0.01
		
	if rtt_ms == 0.0:
		assert_equal(true, false, "Timeout waiting for latency loopback")
		return
	
	print_rich("[color=gray][Latency Test][/color] Loopback Successful! RTT: [b]%.3f ms[/b]" % [rtt_ms])
	assert_equal(rtt_ms > 0, true, "RTT should be positive")

func _on_message(_msg):
	rtt_ms = (Time.get_ticks_usec() - start_time) / 1000.0
	message_looped_back.emit()

func after_test():
	ros_node = null
