extends "res://rclgd_test.gd"


func test_timer_fires_repeatedly():
	var node := make_node("timers_fire")
	var ticks := [0]
	var _timer := node.create_timer(0.05, func(): ticks[0] += 1)
	expect(await wait_until(func(): return ticks[0] >= 3), "timer fired 3 times")


func test_canceled_timer_stops():
	var node := make_node("timers_cancel")
	var ticks := [0]
	var timer := node.create_timer(0.05, func(): ticks[0] += 1)
	expect(await wait_until(func(): return ticks[0] >= 1), "timer fired")

	timer.cancel()
	expect(timer.is_canceled(), "is_canceled")
	await get_tree().create_timer(0.1).timeout # Let in-flight callbacks land
	var after_cancel: int = ticks[0]
	await get_tree().create_timer(0.3).timeout
	expect_eq(ticks[0], after_cancel, "no ticks after cancel")
