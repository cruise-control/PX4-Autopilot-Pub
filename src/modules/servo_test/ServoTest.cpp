/****************************************************************************
 * src/modules/servo_test/ServoTest.cpp  — see ServoTest.hpp
 ****************************************************************************/

#include "ServoTest.hpp"

#include <string.h>
#include <stdio.h>

static inline float clampf(float v, float lo, float hi)
{
	return (v < lo) ? lo : (v > hi) ? hi : v;
}

ServoTest::ServoTest() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

bool ServoTest::init()
{
	ScheduleOnInterval(20_ms);   // 50 Hz control + telemetry loop
	return true;
}

void ServoTest::Run()
{
	if (should_exit()) {
		release_outputs();
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	/* Pick up runtime parameter changes */
	if (_parameter_update_sub.updated()) {
		parameter_update_s pu;
		_parameter_update_sub.copy(&pu);
		updateParams();
	}

	const hrt_abstime now     = hrt_absolute_time();
	const float       rpm_max = static_cast<float>(_param_rpm_max.get());
	const float       step    = static_cast<float>(_param_rpm_step.get());

	/* ---- Encoder input (drain all pending events) ---- */
	rotary_encoder_event_s enc;
	bool ui_dirty = false;
	bool new_esc = false;

	while (_encoder_sub.update(&enc)) {
		/* The encoder driver only publishes on an actual change, so any event
		 * received here is a dial/button action and warrants a redraw. */
		ui_dirty = true;

		if (enc.delta != 0) {
			_target_rpm = clampf(_target_rpm + static_cast<float>(enc.delta) * step, 0.0f, rpm_max);
		}

		/* PUSH toggles the output. A cooldown provides hysteresis so contact
		 * bounce / a long press can't flip the state more than once per press. */
		if (enc.push_rising && (now - _last_toggle > 400_ms)) {
			_last_toggle = now;

			if (_state == State::DISABLED) {
				_state = State::ENABLED;

			} else {
				_state = State::DISABLED;
				release_outputs();
			}
		}

		/* K0 = emergency stop: force output off and zero the target. */
		if (enc.k0_rising) {
			_target_rpm = 0.0f;

			if (_state == State::ENABLED) {
				_state = State::DISABLED;
				release_outputs();
			}
		}
	}

	/* ---- ESC telemetry ---- */
	esc_status_s esc{0};

	if (_esc_status_sub.update(&esc)) {
		_esc      = esc;
		_esc_time = now;
		new_esc = true;
	}

	/* ---- Output ---- */
	if (_state == State::ENABLED) {
		const float value = (rpm_max > 1.0f) ? clampf(_target_rpm / rpm_max, 0.0f, 1.0f) : 0.0f;
		publish_motors(value);
	}

	/* ---- Display: only redraw on a button/encoder action (plus once at
	 * startup). Motor control above still runs every cycle; only the screen is
	 * gated, so ESC telemetry refreshes on dial/button activity rather than
	 * continuously. ---- */
	if (ui_dirty || _first_publish || new_esc) {
		_first_publish = false;
		publish_display();
	}
}

void ServoTest::publish_motors(float value)
{
	int n = _param_num_mot.get();
	n = (n < 1) ? 1 : (n > actuator_test_s::MAX_NUM_MOTORS) ? actuator_test_s::MAX_NUM_MOTORS : n;

	for (int i = 0; i < n; i++) {
		actuator_test_s t{};
		t.timestamp  = hrt_absolute_time();
		t.action     = actuator_test_s::ACTION_DO_CONTROL;
		t.function   = static_cast<uint16_t>(actuator_test_s::FUNCTION_MOTOR1 + i);
		t.value      = value;
		t.timeout_ms = 200;   // motors stop if this module stops publishing (safety)
		_actuator_test_pub.publish(t);
	}
}

void ServoTest::release_outputs()
{
	int n = _param_num_mot.get();
	n = (n < 1) ? 1 : (n > actuator_test_s::MAX_NUM_MOTORS) ? actuator_test_s::MAX_NUM_MOTORS : n;

	for (int i = 0; i < n; i++) {
		actuator_test_s t{};
		t.timestamp  = hrt_absolute_time();
		t.action     = actuator_test_s::ACTION_RELEASE_CONTROL;
		t.function   = static_cast<uint16_t>(actuator_test_s::FUNCTION_MOTOR1 + i);
		t.value      = 0.0f;
		t.timeout_ms = 0;
		_actuator_test_pub.publish(t);
	}
}

void ServoTest::publish_display()
{
	display_command_s d{};
	d.timestamp      = hrt_absolute_time();
	d.numeric_value  = _target_rpm;     // setpoint (large centre area)
	d.decimal_places = 0;
	strncpy(d.units, "RPM", sizeof(d.units));
	d.backlight_on   = true;

	if (_state == State::ENABLED) {
		strncpy(d.status_text, "RUNNING", sizeof(d.status_text));
		d.status_color = 1;   // green

	} else {
		strncpy(d.status_text, "DISABLED", sizeof(d.status_text));
		d.status_color = 0;   // white / blue
	}

	/* Telemetry section from the first ESC report (if fresh) */
	const bool fresh = (_esc_time != 0) && (hrt_absolute_time() - _esc_time < 1_s);

	if (fresh && _esc.esc_count > 0) {
		const esc_report_s &e = _esc.esc[0];
		snprintf(d.info_line1, sizeof(d.info_line1), "RPM %ld", static_cast<long>(e.esc_rpm));
		snprintf(d.info_line2, sizeof(d.info_line2), "%.1fV  %.1fA",
		         static_cast<double>(e.esc_voltage), static_cast<double>(e.esc_current));
		snprintf(d.info_line3, sizeof(d.info_line3), "%dC  PWR %d%%",
		         static_cast<int>(e.esc_temperature - 32.f * 5/9), static_cast<int>(e.esc_power));

	} else {
		strncpy(d.info_line1, "ESC: no telemetry", sizeof(d.info_line1));
		d.info_line2[0] = '\0';
		d.info_line3[0] = '\0';
	}

	_display_command_pub.publish(d);
}

int ServoTest::task_spawn(int argc, char *argv[])
{
	ServoTest *obj = new ServoTest();

	if (!obj) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(obj);
	_task_id = task_id_is_work_queue;

	if (!obj->init()) {
		delete obj;
		_object.store(nullptr);
		_task_id = -1;
		return PX4_ERROR;
	}

	return PX4_OK;
}

int ServoTest::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ServoTest::print_status()
{
	PX4_INFO("state: %s   target: %.0f RPM",
	         (_state == State::ENABLED) ? "ENABLED" : "DISABLED",
	         static_cast<double>(_target_rpm));
	return 0;
}

int ServoTest::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR(
### Description
Bench motor test driven by the EC11 rotary encoder and shown on the ST7789 display.

- The encoder dial sets a target RPM (open-loop: target / SVT_RPM_MAX maps to throttle).
- The encoder PUSH button toggles the output on/off (debounced/hysteresis).
- The K0 button is an emergency stop (output off, target 0).
- ESC telemetry from esc_status is shown in the lower section of the display.

Motors are driven via the actuator_test topic, so this works while disarmed.
)DESCR");

	PRINT_MODULE_USAGE_NAME("servo_test", "command");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int servo_test_main(int argc, char *argv[])
{
	return ServoTest::main(argc, argv);
}
