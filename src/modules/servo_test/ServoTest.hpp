/****************************************************************************
 * src/modules/servo_test/ServoTest.hpp
 *
 * Multi-protocol bench ESC/motor tester for the Crystal NUC-H7xx board.
 *
 * State machine:
 *   CONFIG  — idle; hold K0 2 s to enter SETUP; PUSH to start.
 *   RUNNING — outputs active; dial sets throttle; PUSH=graceful stop; K0=e-stop.
 *   SETUP   — protocol selection overlay (only reachable from CONFIG).
 *
 * Output groups (per-timer, same protocol within a group):
 *   Group A  ch1–4  TIM1  SVT_GRP_A_MODE
 *   Group B  ch5–6  TIM4  SVT_GRP_B_MODE
 *   Group C  ch7–8  TIM2  SVT_GRP_C_MODE
 *   UAVCAN   CAN1         SVT_UAVCAN_EN  (orthogonal to timer groups)
 *
 * Throttle model: normalized 0.0–1.0; same value to all active channels.
 * Ramp: alpha filter (tau = SVT_RAMP_TAU) on start/stop transitions only.
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
#include <uORB/topics/battery_status.h>
#include <uORB/topics/parameter_update.h>
#include <drivers/drv_hrt.h>
#include <lib/parameters/param.h>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>

using namespace time_literals;

class ServoTest : public ModuleBase<ServoTest>,
                  public ModuleParams,
                  public px4::ScheduledWorkItem
{
public:
	ServoTest();
	~ServoTest() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int  print_status() override;

private:
	void Run() override;

	// Output
	void publish_throttle(float value);
	void release_outputs();

	// Protocol configuration (called from SETUP on confirm)
	void apply_group_mode(int group, int mode);

	// State transitions
	void enter_setup();
	void exit_setup(bool commit);
	void start_running();
	void stop_running(bool immediate);

	// Display
	void publish_display();

	// ── State machine ──────────────────────────────────────────────────────
	enum class State     : uint8_t { CONFIG = 0, RUNNING = 1, SETUP = 2 };
	enum class RampState : uint8_t { IDLE = 0, STARTING = 1, STOPPING = 2 };

	// SETUP submenu pages
	enum class SetupPage : uint8_t { MENU = 0, GROUP_A = 1, GROUP_B = 2, GROUP_C = 3, UAVCAN_PAGE = 4 };

	State     _state{State::CONFIG};
	RampState _ramp_state{RampState::IDLE};
	SetupPage _setup_page{SetupPage::MENU};

	// ── Throttle ───────────────────────────────────────────────────────────
	float _target_throttle{0.0f};   // dial setpoint (0–1)
	float _output_throttle{0.0f};   // filtered/actual output (0–1)

	AlphaFilter<float> _ramp_filter{};

	// ── SETUP menu navigation ──────────────────────────────────────────────
	int     _menu_item{0};          // 0–3: highlighted item in SETUP menu
	int     _sub_item{0};           // 0–7: highlighted option in sub-page
	bool    _pending_uavcan{false}; // scratch for UAVCAN sub-page edit
	int32_t _menu_delta_acc{0};     // accumulates encoder delta; fires on ±4 (one detent)

	// ── K0 hold detection (CONFIG state) ──────────────────────────────────
	hrt_abstime _k0_hold_start{0};
	bool        _k0_held{false};
	float       _hold_progress{0.0f};
	bool        _push_pending{false};

	// ── Timing ────────────────────────────────────────────────────────────
	hrt_abstime _last_run{0};
	bool        _first_publish{true};

	// ── Cached sensor data ────────────────────────────────────────────────
	esc_status_s     _esc{};
	hrt_abstime      _esc_time{0};

	battery_status_s _battery{};
	hrt_abstime      _battery_time{0};

	// ── uORB ──────────────────────────────────────────────────────────────
	uORB::Subscription         _encoder_sub{ORB_ID(rotary_encoder_event)};
	uORB::Subscription         _esc_status_sub{ORB_ID(esc_status)};
	uORB::Subscription         _battery_sub{ORB_ID(battery_status)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Publication<actuator_test_s>   _actuator_test_pub{ORB_ID(actuator_test)};
	uORB::Publication<display_command_s> _display_command_pub{ORB_ID(display_command)};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SVT_GRP_A_MODE>)  _param_grp_a_mode,
		(ParamInt<px4::params::SVT_GRP_B_MODE>)  _param_grp_b_mode,
		(ParamInt<px4::params::SVT_GRP_C_MODE>)  _param_grp_c_mode,
		(ParamBool<px4::params::SVT_UAVCAN_EN>)  _param_uavcan_en,
		(ParamBool<px4::params::SVT_RAMP_EN>)    _param_ramp_en,
		(ParamFloat<px4::params::SVT_RAMP_TAU>)  _param_ramp_tau,
		(ParamFloat<px4::params::SVT_THR_STEP>)  _param_thr_step
	)
};
