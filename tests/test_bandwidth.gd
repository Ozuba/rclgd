extends Test

var ros_node: RosNode
var pub: RosPublisher

var test_duration_sec: float = 2.0
var bytes_sent: int = 0
var msg_count: int = 0

func before_test():
	if not rclgd.ok(): rclgd.init([])
	ros_node = RosNode.new()
	ros_node.init("bandwidth_test_node", "")
	pub = ros_node.create_publisher("/bandwidth_load", "std_msgs/msg/ByteMultiArray")

func _execute():
	# Prepare 1MiB payload
	var msg = RosMsg.from_type("std_msgs/msg/ByteMultiArray")
	var data = PackedByteArray()
	data.resize(1024 * 1024) 
	msg.data = data
	
	var start_test_time = Time.get_ticks_msec()
	
	while (Time.get_ticks_msec() - start_test_time) < (test_duration_sec * 1000.0):
		pub.publish(msg)
		bytes_sent += data.size()
		msg_count += 1
		OS.delay_msec(1) 

	_calculate_throughput(test_duration_sec)
	assert_equal(msg_count > 0, true, "Should have sent messages")

func _calculate_throughput(duration: float):
	var mib_total = bytes_sent / (1024.0 * 1024.0)
	var mib_per_sec = mib_total / duration
	print_rich("[color=gray][Bandwidth Test][/color] Throughput: [b]%.2f MiB/s[/b] (%d messages)" % [mib_per_sec, msg_count])

func after_test():
	ros_node = null
