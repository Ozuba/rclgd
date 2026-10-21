extends "res://rclgd_test.gd"

## Classes only rclgd's factories may create.
const FACTORY_MADE := [
	"RosPublisher", "RosSubscriber", "RosClient", "RosRequest", "RosService",
	"RosActionClient", "RosGoalHandle", "RosActionServer", "RosServerGoalHandle",
	"RosTfBroadcaster", "RosTfListener", "RosTimer", "RosLogger",
]
const USER_CREATED := ["RosNode", "RosMsg", "RosQoS"]


func test_factory_made_classes_are_abstract():
	for class_name_ in FACTORY_MADE:
		expect(ClassDB.class_exists(class_name_), class_name_ + " is registered")
		expect(not ClassDB.can_instantiate(class_name_), class_name_ + " cannot be instantiated")


func test_user_created_classes_are_instantiable():
	for class_name_ in USER_CREATED:
		expect(ClassDB.can_instantiate(class_name_), class_name_ + " can be instantiated")
