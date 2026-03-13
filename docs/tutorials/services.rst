Services
========

Services in ROS2 are a request-response mechanism, unlike the publisher-subscriber model which is a many-to-many fire-and-forget communication. Services are better suited for tasks that require a confirmation or a specific result from another node.

In ``rclgd``, you can both create service servers to handle requests and service clients to send requests.

Service Clients
---------------

A Service Client sends a request to a Service Server and waits for a response.

.. code-block:: gdscript

    extends Node

    var node: RosNode
    var client: RosClient

    func _ready():
        node = RosNode.new()
        node.init("my_service_client_node")
        
        # Create a client for the 'add_two_ints' service
        # Service type: example_interfaces/srv/AddTwoInts
        client = node.create_client("add_two_ints", "example_interfaces/srv/AddTwoInts")
        
        # Wait for the service to become available
        if await client.wait_for_service(2.0):
            send_request()
        else:
            print("Service not available")

    func send_request():
        # Create a request message
        var request = client.create_request()
        request.set_member("a", 10)
        request.set_member("b", 5)
        
        # Send the request asynchronously
        # The result will be returned via the 'response_received' signal or as a return value of the async call
        client.call_async(request, _on_response)

    func _on_response(response: RosMsg):
        if response:
            var sum = response.get_member("sum")
            print("Answer: ", sum)
        else:
            print("Service call failed")

Service Servers
---------------

A Service Server listens for requests and provides a response based on the input.

.. code-block:: gdscript

    extends Node

    var node: RosNode
    var service: RosService

    func _ready():
        node = RosNode.new()
        node.init("my_service_server_node")
        
        # Create a service server
        # The callback function will be called when a request is received
        service = node.create_service("add_two_ints", "example_interfaces/srv/AddTwoInts", _handle_add_two_ints)

    func _handle_add_two_ints(request: RosMsg, response: RosMsg):
        # Extract data from the request
        var a = request.get_member("a")
        var b = request.get_member("b")
        
        # Calculate the result and set it in the response
        var sum = a + b
        response.set_member("sum", sum)
        
        print("Received request: %d + %d, responding with %d" % [a, b, sum])
        
        # The response is automatically sent back after the callback returns

Key Concepts
------------

*   **Request/Response**: Every service has a specific request structure and a corresponding response structure, defined in the ``.srv`` file.
*   **Asynchronous Calls**: It is highly recommended to use ``call_async`` in Godot to avoid freezing the main thread while waiting for a network response.
*   **BabelFish Integration**: Like topics, services use ``ros_babel_fish`` for dynamic message handling, meaning you don't need to recompile your GDExtension to use new service types.
