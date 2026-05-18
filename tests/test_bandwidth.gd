extends Test

var ros_node: RosNode
var pub: RosPublisher

# Increase duration slightly for more stable averaging
var test_duration_sec: float = 5.0
var bytes_sent: int = 0
var msg_count: int = 0

func before_test():
	if not rclgd.ok(): rclgd.init([])
	ros_node = RosNode.new()
	ros_node.init("bandwidth_max_node", "")
	
	# OPTIMIZATION: Use a large queue size to prevent RMW dropping packets 
	# if the middleware can't keep up with the loop speed.
	pub = ros_node.create_publisher("/bandwidth_load", "std_msgs/msg/ByteMultiArray")

func _execute():
	# PRE-ALLOCATION: Keep all memory setup outside the loop.
	var msg = RosMsg.from_type("std_msgs/msg/ByteMultiArray")
	var data = PackedByteArray()
	data.resize(1024 * 1024) # 1 MiB
	msg.data = data
	
	var payload_size = data.size()
	var start_test_time = Time.get_ticks_msec()
	var end_time = start_test_time + (test_duration_sec * 1000.0)
	
	# HOT LOOP: No OS.delay_msec(). 
	# This will max out one CPU core to push packets as fast as the RMW allows.
	while Time.get_ticks_msec() < end_time:
		pub.publish(msg)
		msg_count += 1
		# We use a literal or pre-calculated int to avoid calling size() every iteration
		bytes_sent += payload_size 

	var actual_duration = (Time.get_ticks_msec() - start_test_time) / 1000.0
	_calculate_throughput(actual_duration)
	assert(msg_count > 0, "Should have sent messages")

func _calculate_throughput(duration: float):
	var mib_total = bytes_sent / (1024.0 * 1024.0)
	var mib_per_sec = mib_total / duration
	print_rich("[color=cyan][Max Bandwidth Test][/color]")
	print_rich("  - Total Sent: [b]%.2f MiB[/b]" % mib_total)
	print_rich("  - Throughput: [b][color=green]%.2f MiB/s[/color][/b]" % mib_per_sec)
	print_rich("  - Msg Freq:   [b]%.1f Hz[/b]" % (msg_count / duration))

func after_test():
	# Explicit cleanup to ensure the RMW buffers are freed
	pub = null
	ros_node = null
