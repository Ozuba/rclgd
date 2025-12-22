extends Node

# Persistent references to keep the factory-created objects alive
var test_server: RosService
var test_client: RosClient
var ros_node: RosNode

func _ready():
	
	ros_node = RosNode.new()
	ros_node.init("service_node")
	# 1. Initialize the Server via Node Factory
	# Naming follows rclcpp style: (name, type, callback)
	test_server = ros_node.create_service(
		"/test/add_ints", 
		"example_interfaces/srv/AddTwoInts", 
		_on_add_ints_request
	)
	print("[Test] Service Server initialized at /test/add_ints")

	# 2. Initialize the Client via Node Factory
	test_client = ros_node.create_client(
		"/test/add_ints", 
		"example_interfaces/srv/AddTwoInts"
	)
	
	# Run the validation test
	run_validation_test()

## Server Callback: Handles the logic synchronously
func _on_add_ints_request(request: RosMsg, response: RosMsg) -> void:
	# Using dynamic access since no shadow scripts for services exist
	var sum = request.a + request.b
	print("[Server] Received: %d + %d. Calculating..." % [request.a, request.b])
	
	# Populate the response (Writes directly to C++ buffer)
	response.sum = sum

## Client Test Logic: Uses the 'Future' (await) pattern
func run_validation_test():
	print("[Client] Waiting for service...")
	
	# Verify wait_for_service works (simulating rclcpp::wait_for_service)
	if not await test_client.wait_for_service(2.0):
		printerr("[Test Failed] Service timed out or not attached to node!")
		return

	# Create request template from BabelFish
	var req = test_client.create_request()
	req.a = 40
	req.b = 2

	print("[Client] Sending Request: 40 + 2")
	
	# Trigger async call
	test_client.async_send_request(req)
	
	# Await the custom 'request_completed' signal (Our Godot Future)
	var result_msg = await test_client.request_completed
	
	# Validate the data
	if result_msg.sum == 42:
		print("[Test Success] Response received: ", result_msg.sum)
	else:
		printerr("[Test Failed] Unexpected sum: ", result_msg.sum)
