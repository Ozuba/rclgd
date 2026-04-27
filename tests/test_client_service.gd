extends Test

var received_request = false

func _execute():
	var node = RosNode.new()
	node.init("test_client_service_node")
	
	# Server
	var srv = node.create_service("test_service", "example_interfaces/srv/AddTwoInts", func(req, res):
		received_request = true
		res.sum = req.a + req.b
	)
	
	# Client
	var client = node.create_client("test_service", "example_interfaces/srv/AddTwoInts")
	
	var is_ready = client.wait_for_service(2.0)
	assert_equal(is_ready, true, "Service should be ready")
	if not is_ready:
		return
		
	# Create request
	var req = client.create_request()
	req.a = 5
	req.b = 10
	
	# Send request
	var future = client.async_send_request(req)
	
	# Wait for the future to finish
	var res = await future.completed
	
	assert_equal(res != null, true, "Response should not be null")
	if res != null:
		assert_equal(res.sum, 15, "Sum should be 15")
