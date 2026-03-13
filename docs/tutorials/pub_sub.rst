Full Example
------------

Here is a complete GDScript example of a node that increments a counter and publishes it, while also listening to a "chatter" topic.

.. code-block:: gdscript

    extends Node

    var node: RosNode
    var pub: RosPublisher
    var sub: RosSubscriber
    var counter: int = 0

    func _ready():
        # 1. Initialize the node
        node = RosNode.new()
        node.init("pub_sub_example")
        
        # 2. Setup Publisher
        pub = node.create_publisher("counter", "std_msgs/msg/Int32")
        
        # 3. Setup Subscriber
        sub = node.create_subscriber("chatter", "std_msgs/msg/String", _on_chatter)
        
        # 4. Start a timer to publish every second
        var timer = Timer.new()
        add_child(timer)
        timer.wait_time = 1.0
        timer.timeout.connect(_on_timer_timeout)
        timer.start()

    func _on_timer_timeout():
        counter += 1
        var msg = RosMsg.from_type("std_msgs/msg/Int32")
        msg.set_member("data", counter)
        pub.publish(msg)
        print("Published counter: ", counter)

    func _on_chatter(msg: RosMsg):
        var text = msg.get_member("data")
        print("I heard: ", text)
