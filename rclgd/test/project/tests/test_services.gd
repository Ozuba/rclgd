extends "res://rclgd_test.gd"


func test_add_two_ints():
	var node := make_node("services_add")
	var _srv := node.create_service("/rclgd_test/add", "example_interfaces/srv/AddTwoInts",
			func(req, res): res.sum = req.a + req.b)
	var client := node.create_client("/rclgd_test/add", "example_interfaces/srv/AddTwoInts")

	expect(client.wait_for_service(3.0), "service available")
	var req := client.create_request()
	req.a = 5
	req.b = 10
	var res: RosMsg = await client.async_send_request(req).completed
	expect(res != null, "response received")
	if res != null:
		expect_eq(res.sum, 15, "sum")


func test_service_freed_while_handling_request():
	var node := make_node("services_free")
	var handling := []
	var srv := node.create_service("/rclgd_test/slow_add", "example_interfaces/srv/AddTwoInts",
			func(req, res):
				handling.append(true)
				OS.delay_msec(100) # On a ROS thread: the main thread frees the service meanwhile
				res.sum = req.a + req.b)
	var client := node.create_client("/rclgd_test/slow_add", "example_interfaces/srv/AddTwoInts")
	expect(client.wait_for_service(3.0), "service available")

	var req := client.create_request()
	req.a = 2
	req.b = 3
	var request := client.async_send_request(req)
	expect(await wait_until(func(): return not handling.is_empty()), "request being handled")
	srv = null
	expect(await wait_until(func(): return request.is_completed()), "response still delivered")
	if request.is_completed():
		expect_eq(request.get_response().sum, 5, "sum")
