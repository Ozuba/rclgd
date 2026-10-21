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
