/****************************************************************************
 * src/modules/servo_test/ServoTest.cpp  — see ServoTest.hpp
 ****************************************************************************/

#include "ServoTest.hpp"

#include <string.h>
#include <stdio.h>
#include <math.h>

// ── Helpers ────────────────────────────────────────────────────────────────

static inline float clampf(float v, float lo, float hi)
{
	return (v < lo) ? lo : (v > hi) ? hi : v;
}

// mode enum → PWM_MAIN_TIMx integer value (negative=DShot rate, positive=PWM Hz, 0=disabled)
static int32_t mode_to_tim_value(int mode)
{
	switch (mode) {
	case 0:  return  0;    // disabled
	case 1:  return -5;    // DShot150
	case 2:  return -4;    // DShot300
	case 3:  return -3;    // DShot600
	case 4:  return  50;   // PWM 50 Hz
	case 5:  return  100;  // PWM 100 Hz
	case 6:  return  200;  // PWM 200 Hz
	case 7:  return  400;  // PWM 400 Hz
	default: return  0;
	}
}

static const char *mode_name(int mode)
{
	switch (mode) {
	case 0:  return "disabled";
	case 1:  return "DShot150";
	case 2:  return "DShot300";
	case 3:  return "DShot600";
	case 4:  return "PWM  50Hz";
	case 5:  return "PWM 100Hz";
	case 6:  return "PWM 200Hz";
	case 7:  return "PWM 400Hz";
	default: return "?";
	}
}

static bool mode_is_dshot(int mode) { return mode >= 1 && mode <= 3; }
static bool mode_is_pwm(int mode)   { return mode >= 4 && mode <= 7; }

// Group index → first and last channel (1-based)
static void group_channels(int group, int &first, int &last)
{
	switch (group) {
	case 0: first = 1; last = 4; break;  // Group A / TIM1
	case 1: first = 5; last = 6; break;  // Group B / TIM4
	case 2: first = 7; last = 8; break;  // Group C / TIM2
	default: first = last = 0; break;
	}
}

// ── ServoTest ──────────────────────────────────────────────────────────────

ServoTest::ServoTest() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

bool ServoTest::init()
{
	ScheduleOnInterval(20_ms);   // 50 Hz
	return true;
}

void ServoTest::Run()
{
	if (should_exit()) {
		stop_running(/*immediate=*/true);
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s pu;
		_parameter_update_sub.copy(&pu);
		updateParams();
	}

	const hrt_abstime now = hrt_absolute_time();
	const float dt = (_last_run == 0) ? 0.02f : clampf((now - _last_run) * 1e-6f, 0.001f, 0.1f);
	_last_run = now;

	// ── Sensor subscriptions ────────────────────────────────────────────
	esc_status_s esc_msg{};

	if (_esc_status_sub.update(&esc_msg)) {
		_esc      = esc_msg;
		_esc_time = now;
	}

	battery_status_s bat_msg{};

	if (_battery_sub.update(&bat_msg)) {
		_battery      = bat_msg;
		_battery_time = now;
	}

	// ── Encoder events ──────────────────────────────────────────────────
	rotary_encoder_event_s enc;
	bool ui_dirty = _first_publish;

	while (_encoder_sub.update(&enc)) {
		ui_dirty = true;

		switch (_state) {

		case State::CONFIG:
			// Dial pre-sets target throttle so the user can see what they'll start at
			if (enc.delta != 0) {
				_target_throttle = clampf(_target_throttle + static_cast<float>(enc.delta) * _param_thr_step.get(), 0.0f, 1.0f);
			}

			// PUSH → arm on press, start on release
			if (enc.push_rising) {
				_push_pending = true;
			}

			if (enc.push_falling && _push_pending) {
				_push_pending = false;
				start_running();
			}

			// K0 hold tracking
			if (enc.k0_rising) {
				_push_pending  = false;  // cancel pending start when K0 is engaged
				_k0_held       = true;
				_k0_hold_start = now;
				_hold_progress = 0.0f;
			}

			if (enc.k0_falling) {
				_k0_held       = false;
				_hold_progress = 0.0f;
			}

			break;

		case State::RUNNING:
			if (enc.delta != 0) {
				_target_throttle = clampf(_target_throttle + static_cast<float>(enc.delta) * _param_thr_step.get(), 0.0f, 1.0f);
			}

			// PUSH → graceful stop
			if (enc.push_rising) {
				stop_running(/*immediate=*/false);
			}

			// K0 → immediate e-stop
			if (enc.k0_rising) {
				stop_running(/*immediate=*/true);
			}

			break;

		case State::SETUP:
			if (_setup_page == SetupPage::MENU) {
				if (enc.delta != 0) {
					_menu_delta_acc += enc.delta;
					while (_menu_delta_acc >=  4) { _menu_delta_acc -= 4; _menu_item = (_menu_item + 1) % 4; }
					while (_menu_delta_acc <= -4) { _menu_delta_acc += 4; _menu_item = (_menu_item + 3) % 4; }
				}

				if (enc.push_rising) {
					// Enter the highlighted sub-page
					_menu_delta_acc = 0;
					switch (_menu_item) {
					case 0: _setup_page = SetupPage::GROUP_A;
						_sub_item = _param_grp_a_mode.get(); break;
					case 1: _setup_page = SetupPage::GROUP_B;
						_sub_item = _param_grp_b_mode.get(); break;
					case 2: _setup_page = SetupPage::GROUP_C;
						_sub_item = _param_grp_c_mode.get(); break;
					case 3: _setup_page = SetupPage::UAVCAN_PAGE;
						_pending_uavcan = _param_uavcan_en.get(); break;
					}
				}

				if (enc.k0_rising) {
					exit_setup(/*commit=*/false);
				}

			} else if (_setup_page == SetupPage::UAVCAN_PAGE) {
				if (enc.delta != 0) {
					_menu_delta_acc += enc.delta;
					while (_menu_delta_acc >=  4) { _menu_delta_acc -= 4; _pending_uavcan = !_pending_uavcan; }
					while (_menu_delta_acc <= -4) { _menu_delta_acc += 4; _pending_uavcan = !_pending_uavcan; }
				}

				if (enc.push_rising) {
					exit_setup(/*commit=*/true);
				}

				if (enc.k0_rising) {
					_setup_page = SetupPage::MENU;
				}

			} else {
				// Protocol sub-page (GROUP_A / B / C)
				if (enc.delta != 0) {
					_menu_delta_acc += enc.delta;
					while (_menu_delta_acc >=  4) { _menu_delta_acc -= 4; _sub_item = (_sub_item + 1) % 8; }
					while (_menu_delta_acc <= -4) { _menu_delta_acc += 4; _sub_item = (_sub_item + 7) % 8; }
				}

				if (enc.push_rising) {
					int group = static_cast<int>(_setup_page) - 1; // SetupPage::GROUP_A=1 → group 0
					apply_group_mode(group, _sub_item);
					_setup_page     = SetupPage::MENU;
					_menu_delta_acc = 0;
				}

				if (enc.k0_rising) {
					_setup_page     = SetupPage::MENU;
					_menu_delta_acc = 0;
				}
			}

			break;
		}
	}

	// ── K0 hold progress (CONFIG state only) ────────────────────────────
	if (_state == State::CONFIG && _k0_held) {
		_hold_progress = clampf((now - _k0_hold_start) * 1e-6f / 2.0f, 0.0f, 1.0f);
		ui_dirty = true;

		if (_hold_progress >= 1.0f) {
			_k0_held       = false;
			_hold_progress = 0.0f;
			enter_setup();
		}
	}

	// ── Ramp / throttle output ───────────────────────────────────────────
	if (_state == State::RUNNING) {
		if (_ramp_state == RampState::STARTING) {
			_output_throttle = _ramp_filter.update(_target_throttle, dt);

			if (fabsf(_output_throttle - _target_throttle) < 0.01f) {
				_ramp_state = RampState::IDLE;
			}

		} else if (_ramp_state == RampState::STOPPING) {
			_output_throttle = _ramp_filter.update(0.0f, dt);

			if (_output_throttle < 0.01f) {
				_target_throttle = 0.0f;
				_output_throttle = 0.0f;
				release_outputs();
				_state      = State::CONFIG;
				_ramp_state = RampState::IDLE;
				ui_dirty    = true;
			}

		} else {
			// Steady RUNNING — direct, no filter
			_output_throttle = _target_throttle;
		}

		if (_state == State::RUNNING) {
			publish_throttle(_output_throttle);
		}
	}

	// ── Display ─────────────────────────────────────────────────────────
	if (ui_dirty || (_esc_time == now) || (_battery_time == now)) {
		_first_publish = false;
		publish_display();
	}
}

// ── Output ────────────────────────────────────────────────────────────────

void ServoTest::publish_throttle(float value)
{
	const int32_t modes[3] = {
		_param_grp_a_mode.get(),
		_param_grp_b_mode.get(),
		_param_grp_c_mode.get()
	};

	// Timer groups: publish actuator_test for each active channel
	for (int g = 0; g < 3; g++) {
		if (modes[g] == 0) { continue; }  // disabled

		int first, last;
		group_channels(g, first, last);

		for (int ch = first; ch <= last; ch++) {
			actuator_test_s t{};
			t.timestamp  = hrt_absolute_time();
			t.action     = actuator_test_s::ACTION_DO_CONTROL;
			t.function   = static_cast<uint16_t>(actuator_test_s::FUNCTION_MOTOR1 + (ch - 1));
			t.value      = value;
			t.timeout_ms = 200;
			_actuator_test_pub.publish(t);
		}
	}

	// UAVCAN: publish Motor1–8; the UAVCAN driver routes via UAVCAN_EC_FUNC1-8
	if (_param_uavcan_en.get()) {
		for (int i = 0; i < 8; i++) {
			actuator_test_s t{};
			t.timestamp  = hrt_absolute_time();
			t.action     = actuator_test_s::ACTION_DO_CONTROL;
			t.function   = static_cast<uint16_t>(actuator_test_s::FUNCTION_MOTOR1 + i);
			t.value      = value;
			t.timeout_ms = 200;
			_actuator_test_pub.publish(t);
		}
	}
}

void ServoTest::release_outputs()
{
	const int32_t modes[3] = {
		_param_grp_a_mode.get(),
		_param_grp_b_mode.get(),
		_param_grp_c_mode.get()
	};

	for (int g = 0; g < 3; g++) {
		if (modes[g] == 0) { continue; }

		int first, last;
		group_channels(g, first, last);

		for (int ch = first; ch <= last; ch++) {
			actuator_test_s t{};
			t.timestamp  = hrt_absolute_time();
			t.action     = actuator_test_s::ACTION_RELEASE_CONTROL;
			t.function   = static_cast<uint16_t>(actuator_test_s::FUNCTION_MOTOR1 + (ch - 1));
			t.value      = 0.0f;
			t.timeout_ms = 0;
			_actuator_test_pub.publish(t);
		}
	}

	if (_param_uavcan_en.get()) {
		for (int i = 0; i < 8; i++) {
			actuator_test_s t{};
			t.timestamp  = hrt_absolute_time();
			t.action     = actuator_test_s::ACTION_RELEASE_CONTROL;
			t.function   = static_cast<uint16_t>(actuator_test_s::FUNCTION_MOTOR1 + i);
			t.value      = 0.0f;
			t.timeout_ms = 0;
			_actuator_test_pub.publish(t);
		}
	}
}

// ── Protocol configuration ────────────────────────────────────────────────

void ServoTest::apply_group_mode(int group, int mode)
{
	// Map group index → PWM_MAIN_TIM param name
	static const char *tim_params[] = { "PWM_MAIN_TIM0", "PWM_MAIN_TIM1", "PWM_MAIN_TIM2" };

	if (group < 0 || group > 2) { return; }

	// Write timer protocol param
	int32_t tim_val = mode_to_tim_value(mode);
	param_t p = param_find(tim_params[group]);

	if (p != PARAM_INVALID) {
		param_set(p, &tim_val);
	}

	// Auto-fill channel min/max/dis/fail based on protocol
	int first, last;
	group_channels(group, first, last);

	int32_t min_v, max_v, dis_v, fail_v;

	if (mode_is_dshot(mode)) {
		min_v = 0; max_v = 1999; dis_v = 0; fail_v = 0;

	} else if (mode_is_pwm(mode)) {
		min_v = 1100; max_v = 1900; dis_v = 1100; fail_v = 1100;

	} else {
		// disabled — skip channel param update
		// Update the SVT group param to reflect the new mode
		const char *svt_params[] = { "SVT_GRP_A_MODE", "SVT_GRP_B_MODE", "SVT_GRP_C_MODE" };
		int32_t m = mode;
		p = param_find(svt_params[group]);
		if (p != PARAM_INVALID) { param_set(p, &m); }
		return;
	}

	char name[20];

	for (int ch = first; ch <= last; ch++) {
		snprintf(name, sizeof(name), "PWM_MAIN_MIN%d", ch);
		p = param_find(name);
		if (p != PARAM_INVALID) { param_set(p, &min_v); }

		snprintf(name, sizeof(name), "PWM_MAIN_MAX%d", ch);
		p = param_find(name);
		if (p != PARAM_INVALID) { param_set(p, &max_v); }

		snprintf(name, sizeof(name), "PWM_MAIN_DIS%d", ch);
		p = param_find(name);
		if (p != PARAM_INVALID) { param_set(p, &dis_v); }

		snprintf(name, sizeof(name), "PWM_MAIN_FAIL%d", ch);
		p = param_find(name);
		if (p != PARAM_INVALID) { param_set(p, &fail_v); }
	}

	// Update the SVT group param so it persists
	const char *svt_params[] = { "SVT_GRP_A_MODE", "SVT_GRP_B_MODE", "SVT_GRP_C_MODE" };
	int32_t m = mode;
	p = param_find(svt_params[group]);

	if (p != PARAM_INVALID) { param_set(p, &m); }

	updateParams();
}

// ── State transitions ─────────────────────────────────────────────────────

void ServoTest::start_running()
{
	_ramp_filter.reset(0.0f);
	_ramp_filter.setParameters(0.02f, _param_ramp_tau.get());

	_ramp_state      = _param_ramp_en.get() ? RampState::STARTING : RampState::IDLE;
	_output_throttle = 0.0f;
	_state           = State::RUNNING;
}

void ServoTest::stop_running(bool immediate)
{
	if (immediate || !_param_ramp_en.get()) {
		release_outputs();
		_target_throttle = 0.0f;
		_output_throttle = 0.0f;
		_ramp_state      = RampState::IDLE;
		_state           = State::CONFIG;

	} else {
		// Graceful ramp-down — Run() will call release_outputs() when done
		_ramp_filter.reset(_output_throttle);
		_ramp_filter.setParameters(0.02f, _param_ramp_tau.get());
		_ramp_state = RampState::STOPPING;
	}
}

void ServoTest::enter_setup()
{
	_setup_page     = SetupPage::MENU;
	_menu_item      = 0;
	_menu_delta_acc = 0;
	_push_pending   = false;
	_state          = State::SETUP;
}

void ServoTest::exit_setup(bool commit)
{
	_menu_delta_acc = 0;

	if (commit && _setup_page == SetupPage::UAVCAN_PAGE) {
		bool en = _pending_uavcan;
		param_t p = param_find("SVT_UAVCAN_EN");

		if (p != PARAM_INVALID) { param_set(p, &en); }

		updateParams();
	}

	_setup_page = SetupPage::MENU;
	_state      = State::CONFIG;
}

// ── Display ───────────────────────────────────────────────────────────────

void ServoTest::publish_display()
{
	display_command_s d{};
	d.timestamp    = hrt_absolute_time();
	d.backlight_on = true;

	// Supply voltage/current from INA238 (battery_status)
	char supply_buf[20];
	const bool bat_fresh = (_battery_time != 0) && (hrt_absolute_time() - _battery_time < 2_s);

	if (bat_fresh && _battery.connected) {
		snprintf(supply_buf, sizeof(supply_buf), "%.1fV %.1fA",
			 static_cast<double>(_battery.voltage_v),
			 static_cast<double>(_battery.current_a));

	} else {
		snprintf(supply_buf, sizeof(supply_buf), "--V --A");
	}

	const int32_t mode_a = _param_grp_a_mode.get();
	const int32_t mode_b = _param_grp_b_mode.get();
	const int32_t mode_c = _param_grp_c_mode.get();

	switch (_state) {

	case State::CONFIG:
		if (_k0_held && _hold_progress > 0.0f) {
			// K0 hold in progress — show progress bar in status
			snprintf(d.status_text, sizeof(d.status_text), "HOLD K0...");
			d.status_color = 2;  // yellow

			// ASCII progress bar in info_line3: [========  ] XX%
			int filled = static_cast<int>(_hold_progress * 10.0f);
			char bar[12];
			for (int i = 0; i < 10; i++) { bar[i] = (i < filled) ? '=' : ' '; }
			bar[10] = '\0';
			snprintf(d.info_line3, sizeof(d.info_line3), "[%s] %d%%", bar,
				 static_cast<int>(_hold_progress * 100.0f));

			// Keep group summary in lines 1–2
			snprintf(d.info_line1, sizeof(d.info_line1), "A:%-9s B:%s",
				 mode_name(mode_a), mode_name(mode_b));
			snprintf(d.info_line2, sizeof(d.info_line2), "C:%-9s CAN:%s",
				 mode_name(mode_c), _param_uavcan_en.get() ? "on" : "off");

		} else {
			snprintf(d.status_text, sizeof(d.status_text), "CONFIG  %s", supply_buf);
			d.status_color = 0;  // white

			snprintf(d.info_line1, sizeof(d.info_line1), "A: %-9s", mode_name(mode_a));
			snprintf(d.info_line2, sizeof(d.info_line2), "B:%-9s C:%s",
				 mode_name(mode_b), mode_name(mode_c));
			snprintf(d.info_line3, sizeof(d.info_line3), "CAN:%-3s [K0=setup]",
				 _param_uavcan_en.get() ? "on" : "off");
		}

		// Centre: pre-set throttle
		d.numeric_value  = _target_throttle * 100.0f;
		d.decimal_places = 2;
		strncpy(d.units, "% SET", sizeof(d.units));
		break;

	case State::RUNNING:
		snprintf(d.status_text, sizeof(d.status_text), "RUNNING %s", supply_buf);
		d.status_color   = 1;  // green
		d.numeric_value  = ((_ramp_state == RampState::STARTING) ? _target_throttle : _output_throttle) * 100.0f;
		d.decimal_places = 2;
		strncpy(d.units, "%", sizeof(d.units));

		{
			const bool esc_fresh = (_esc_time != 0) && (hrt_absolute_time() - _esc_time < 1_s);

			if (esc_fresh && _esc.esc_count > 0) {
				const esc_report_s &e = _esc.esc[0];
				snprintf(d.info_line1, sizeof(d.info_line1), "RPM %ld",
					 static_cast<long>(e.esc_rpm));
				snprintf(d.info_line2, sizeof(d.info_line2), "%.1fV  %.1fA",
					 static_cast<double>(e.esc_voltage),
					 static_cast<double>(e.esc_current));
				snprintf(d.info_line3, sizeof(d.info_line3), "%dC  PWR %d%%",
					 static_cast<int>(e.esc_temperature - 32.f * 5 / 9),
					 static_cast<int>(e.esc_power));

			} else {
				strncpy(d.info_line1, "ESC: no telemetry", sizeof(d.info_line1));
				d.info_line2[0] = '\0';
				d.info_line3[0] = '\0';
			}
		}

		break;

	case State::SETUP:
		d.status_color = 2;  // yellow

		if (_setup_page == SetupPage::MENU) {
			snprintf(d.status_text, sizeof(d.status_text), "SETUP  [K0=exit]");

			// 4 items, 3 lines — scroll window so selected item is always visible
			static const char *labels[] = { "A", "B", "C", "CAN" };
			const int32_t modes_arr[] = { mode_a, mode_b, mode_c, -1 };

			// Determine the first item to display (keep selected in view)
			int start = _menu_item - 1;
			if (start < 0) { start = 0; }
			if (start > 1) { start = 1; }

			char *lines[] = { d.info_line1, d.info_line2, d.info_line3 };

			for (int i = 0; i < 3; i++) {
				int item = start + i;

				if (item > 3) { lines[i][0] = '\0'; continue; }

				const char *sel = (item == _menu_item) ? ">" : " ";
				const char *val = (item == 3)
						  ? (_param_uavcan_en.get() ? "enabled" : "disabled")
						  : mode_name(static_cast<int>(modes_arr[item]));

				snprintf(lines[i], 20, "%s%-3s: %s", sel, labels[item], val);
			}

			// Centre shows current item value for readability
			d.numeric_value  = _menu_item;
			d.decimal_places = 0;
			strncpy(d.units, "ITEM", sizeof(d.units));

		} else if (_setup_page == SetupPage::UAVCAN_PAGE) {
			snprintf(d.status_text, sizeof(d.status_text), "UAVCAN [K0=back]");
			snprintf(d.info_line1, sizeof(d.info_line1), "%s disabled",
				 !_pending_uavcan ? ">" : " ");
			snprintf(d.info_line2, sizeof(d.info_line2), "%s enabled",
				 _pending_uavcan ? ">" : " ");
			d.info_line3[0]  = '\0';
			d.numeric_value  = 0;
			d.decimal_places = 0;
			strncpy(d.units, "", sizeof(d.units));

		} else {
			// Protocol sub-page (GROUP_A/B/C)
			int group     = static_cast<int>(_setup_page) - 1;
			const char *group_names[] = { "GROUP A", "GROUP B", "GROUP C" };
			snprintf(d.status_text, sizeof(d.status_text), "%s [K0=back]",
				 group_names[group]);

			// 8 options, 3 lines — scroll window around _sub_item
			int start = _sub_item - 1;
			if (start < 0) { start = 0; }
			if (start > 5) { start = 5; }

			char *lines[] = { d.info_line1, d.info_line2, d.info_line3 };

			for (int i = 0; i < 3; i++) {
				int opt = start + i;

				if (opt > 7) { lines[i][0] = '\0'; continue; }

				const char *sel = (opt == _sub_item) ? ">" : " ";
				snprintf(lines[i], 20, "%s%s", sel, mode_name(opt));
			}

			d.numeric_value  = _sub_item;
			d.decimal_places = 0;
			strncpy(d.units, "MODE", sizeof(d.units));
		}

		break;
	}

	_display_command_pub.publish(d);
}

// ── Module boilerplate ────────────────────────────────────────────────────

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
	const char *state_names[] = { "CONFIG", "RUNNING", "SETUP" };
	PX4_INFO("state: %s   throttle: %.1f%%   ramp: %s",
		 state_names[static_cast<int>(_state)],
		 static_cast<double>(_output_throttle * 100.0f),
		 _param_ramp_en.get() ? "on" : "off");
	PX4_INFO("groups: A=%s B=%s C=%s  UAVCAN=%s",
		 mode_name(_param_grp_a_mode.get()),
		 mode_name(_param_grp_b_mode.get()),
		 mode_name(_param_grp_c_mode.get()),
		 _param_uavcan_en.get() ? "enabled" : "disabled");
	return 0;
}

int ServoTest::print_usage(const char *reason)
{
	if (reason) { PX4_WARN("%s", reason); }

	PRINT_MODULE_DESCRIPTION(
		R"DESCR(
### Description
Multi-protocol bench ESC/motor tester driven by the EC11 rotary encoder and
shown on the ST7789 display. Supports DShot (150/300/600), PWM (50–400 Hz),
and UAVCAN output simultaneously across three independent timer groups.

State machine:
  CONFIG  — hold K0 2 s to enter SETUP; PUSH to start output.
  RUNNING — dial sets normalized throttle (0–100%); PUSH=graceful stop; K0=e-stop.
  SETUP   — select per-group protocol and UAVCAN enable; K0 exits.

Outputs are driven via the actuator_test topic (works while disarmed).
Supply voltage and current are shown from the INA238 power monitor.
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
