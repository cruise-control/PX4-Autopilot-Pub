/****************************************************************************
 * src/modules/servo_test/ServoTest.hpp
 *
 * Bench motor-test module for the Crystal NUC-H7xx board.
 *
 *  - Subscribes to the EC11 rotary encoder (rotary_encoder_event) to set a
 *    target RPM and to toggle the output on/off (shaft button, with a
 *    debounce/cooldown hysteresis). The K0 button is an emergency stop.
 *  - Subscribes to esc_status for live ESC telemetry.
 *  - Drives the motors via the actuator_test topic (works while disarmed).
 *  - Publishes display_command so the ST7789 shows the state, the target RPM
 *    (setpoint) and the live ESC telemetry.
 *
 * BSD 3-Clause License.
 ****************************************************************************/

#pragma once

#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/rotary_encoder_event.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/esc_report.h>
#include <uORB/topics/actuator_test.h>
#include <uORB/topics/display_command.h>
#include <uORB/topics/parameter_update.h>
#include <drivers/drv_hrt.h>
#include <lib/parameters/param.h>

using namespace time_literals;

class ServoTest : public ModuleBase<ServoTest>,
                  public ModuleParams,
                  public px4::ScheduledWorkItem
{
public:
	ServoTest();
	~ServoTest() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int  print_status() override;

private:
	void Run() override;

	void publish_motors(float value);   ///< actuator_test DO_CONTROL for motors 1..N
	void release_outputs();             ///< actuator_test RELEASE_CONTROL for motors 1..N
	void publish_display();

	enum class State : uint8_t { DISABLED = 0, ENABLED = 1 };
	State       _state{State::DISABLED};

	float       _target_rpm{0.0f};
	hrt_abstime _last_toggle{0};
	bool        _first_publish{true};   ///< force one draw on startup

	/* Cached ESC telemetry */
	esc_status_s _esc{};
	hrt_abstime  _esc_time{0};

	uORB::Subscription _encoder_sub{ORB_ID(rotary_encoder_event)};
	uORB::Subscription _esc_status_sub{ORB_ID(esc_status)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Publication<actuator_test_s>   _actuator_test_pub{ORB_ID(actuator_test)};
	uORB::Publication<display_command_s> _display_command_pub{ORB_ID(display_command)};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SVT_RPM_STEP>) _param_rpm_step,
		(ParamInt<px4::params::SVT_RPM_MAX>)  _param_rpm_max,
		(ParamInt<px4::params::SVT_NUM_MOT>)  _param_num_mot
	)
};
