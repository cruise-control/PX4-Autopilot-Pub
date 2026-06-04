############################################################################
# src/modules/servo_test/CMakeLists.txt
############################################################################

px4_add_module(
	MODULE modules__servo_test
	MAIN servo_test
	SRCS
		ServoTest.cpp
	DEPENDS
		px4_work_queue
	MODULE_CONFIG
		module.yaml
)
