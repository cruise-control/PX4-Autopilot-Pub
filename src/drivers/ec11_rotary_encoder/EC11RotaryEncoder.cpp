/****************************************************************************
 * drivers/ec11_rotary_encoder/EC11RotaryEncoder.cpp
 *
 * PX4 driver for the EC11 rotary encoder + push button + K0 back button
 * as present on the 2.4" TFT/EC11 combo board (B0GW29KWS9).
 *
 * Quadrature decode:
 *   The EC11 produces two square waves (A & B) 90° apart.
 *   We sample AB on every edge and compare against the previous state
 *   using the standard 4-state Gray-code table.
 *
 *   State table (previous → current → direction):
 *     00→01, 01→11, 11→10, 10→00  →  CW  (+1)
 *     00→10, 10→11, 11→01, 01→00  →  CCW (−1)
 *
 * Publishing strategy:
 *   ISRs set volatile flags and wake the work-queue task.
 *   The work task does debouncing, state accumulation, and publishes
 *   a single rotary_encoder_event_s per cycle.
 *
 * BSD 3-Clause License – see LICENSE for details.
 ****************************************************************************/

#include "EC11RotaryEncoder.hpp"

#include <px4_platform_common/log.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/posix.h>

#include <board_config.h>
#include <stm32_gpio.h>   // stm32_gpiosetevent / stm32_gpioread  (NuttX / STM32)

/*
 * Gray-code quadrature table.
 * Index: (prev_AB << 2) | curr_AB
 * Value: +1 CW, -1 CCW, 0 invalid/no-change
 */
static constexpr int8_t kQuadTable[16] = {
/*        curr: 00  01  10  11          prev */
/* 00 */      0,  -1, 1,  0,
/* 01 */      1,  0,  0,  -1,
/* 10 */      -1,  0,  0, 1,
/* 11 */      0, 1,  -1,  0
};

/* -------------------------------------------------------------------------
 * Construction / destruction
 * -------------------------------------------------------------------------*/
EC11RotaryEncoder::EC11RotaryEncoder() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

EC11RotaryEncoder::~EC11RotaryEncoder()
{
	/* Disable all GPIO interrupts we registered */
	stm32_gpiosetevent(_param_gpio_a.get(),    false, false, false, nullptr, nullptr);
	stm32_gpiosetevent(_param_gpio_b.get(),    false, false, false, nullptr, nullptr);
	stm32_gpiosetevent(_param_gpio_push.get(), false, false, false, nullptr, nullptr);
	stm32_gpiosetevent(_param_gpio_k0.get(),   false, false, false, nullptr, nullptr);

	ScheduleClear();
}

/* -------------------------------------------------------------------------
 * ModuleBase entry points
 * -------------------------------------------------------------------------*/
int EC11RotaryEncoder::task_spawn(int argc, char *argv[])
{
	EC11RotaryEncoder *obj = new EC11RotaryEncoder();

	if (!obj) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	if (!obj->init()) {
		delete obj;
		return PX4_ERROR;
	}

	_object.store(obj);
	_task_id = task_id_is_work_queue;
	return PX4_OK;
}

int EC11RotaryEncoder::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int EC11RotaryEncoder::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver for the EC11 quadrature rotary encoder, encoder push-button (PUSH),
and independent back/menu button (K0) found on the 2.4" TFT + EC11 combo
board (B0GW29KWS9).

Publishes `rotary_encoder_event` uORB topic.

### Parameters
EC11_GPIO_PIN_A   – GPIO configuration word for phase-A line
EC11_GPIO_PIN_B   – GPIO configuration word for phase-B line
EC11_GPIO_PUSH – GPIO configuration word for PUSH button
EC11_GPIO_K0  – GPIO configuration word for K0 button
EC11_DEBOUNCE_US  – Button debounce period [µs] (default 5000)
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ec11_rotary_encoder", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

/* -------------------------------------------------------------------------
 * Initialisation
 * -------------------------------------------------------------------------*/
bool EC11RotaryEncoder::init()
{
	updateParams();

	/* Configure pins as inputs with pull-ups (board_config.h may pre-configure) */
	stm32_configgpio(_param_gpio_a.get());
	stm32_configgpio(_param_gpio_b.get());
	stm32_configgpio(_param_gpio_push.get());
	stm32_configgpio(_param_gpio_k0.get());

	/* Read initial AB state */
	bool a = stm32_gpioread(_param_gpio_a.get());
	bool b = stm32_gpioread(_param_gpio_b.get());
	_ab_state = (a ? 2u : 0u) | (b ? 1u : 0u);

	/* Install GPIO edge interrupts — both edges for encoder, both for buttons */
	if (stm32_gpiosetevent(_param_gpio_a.get(),    true, true, false, encoder_a_isr, this) != OK ||
	    stm32_gpiosetevent(_param_gpio_b.get(),    true, true, false, encoder_b_isr, this) != OK ||
	    stm32_gpiosetevent(_param_gpio_push.get(), true, true, false, push_isr,     this) != OK ||
	    stm32_gpiosetevent(_param_gpio_k0.get(),   true, true, false, k0_isr,       this) != OK) {
		PX4_ERR("GPIO interrupt registration failed");
		return false;
	}

	/* Schedule the work-queue polling task at ~100 Hz as safety net */
	ScheduleOnInterval(10_ms);

	PX4_INFO("EC11 encoder driver started – A=0x%08X B=0x%08X PUSH=0x%08X K0=0x%08X",
	         (unsigned int)_param_gpio_a.get(), (unsigned int)_param_gpio_b.get(),
	         (unsigned int)_param_gpio_push.get(), (unsigned int)_param_gpio_k0.get());
	return true;
}

/* -------------------------------------------------------------------------
 * GPIO ISR trampolines (static → non-static)
 * -------------------------------------------------------------------------*/
int EC11RotaryEncoder::encoder_a_isr(int irq, void *context, void *arg)
{
	auto *self = reinterpret_cast<EC11RotaryEncoder *>(arg);
	bool a = stm32_gpioread(self->_param_gpio_a.get());
	bool b = stm32_gpioread(self->_param_gpio_b.get());
	self->process_encoder_step(a, b);
	return OK;
}

int EC11RotaryEncoder::encoder_b_isr(int irq, void *context, void *arg)
{
	auto *self = reinterpret_cast<EC11RotaryEncoder *>(arg);
	bool a = stm32_gpioread(self->_param_gpio_a.get());
	bool b = stm32_gpioread(self->_param_gpio_b.get());
	self->process_encoder_step(a, b);
	return OK;
}

int EC11RotaryEncoder::push_isr(int irq, void *context, void *arg)
{
	reinterpret_cast<EC11RotaryEncoder *>(arg)->_push_changed = true;
	return OK;
}

int EC11RotaryEncoder::k0_isr(int irq, void *context, void *arg)
{
	reinterpret_cast<EC11RotaryEncoder *>(arg)->_k0_changed = true;
	return OK;
}

/* -------------------------------------------------------------------------
 * Quadrature decode (called from ISR context)
 * -------------------------------------------------------------------------*/
void EC11RotaryEncoder::process_encoder_step(bool a_level, bool b_level)
{
	uint8_t curr = (a_level ? 2u : 0u) | (b_level ? 1u : 0u);
	uint8_t idx  = static_cast<uint8_t>((_ab_state << 2u) | curr);
	int8_t  step = kQuadTable[idx & 0x0Fu];

	if (step != 0) {
		_position      += step*4;
		_delta         += step*4;
		_encoder_changed = true;
	}

	_ab_state = curr;
}

/* -------------------------------------------------------------------------
 * Button debounce helper
 * Returns true when a stable edge is detected.
 * -------------------------------------------------------------------------*/
bool EC11RotaryEncoder::debounce_button(hrt_abstime &last_change,
                                        bool        &last_state,
                                        bool         new_state,
                                        hrt_abstime  now)
{
	if (new_state == last_state) {
		return false;  // no change
	}

	const uint64_t debounce = static_cast<uint64_t>(_param_debounce_us.get());

	if ((now - last_change) < debounce) {
		return false;  // too soon – glitch
	}

	last_change = now;
	last_state  = new_state;
	return true;
}

/* -------------------------------------------------------------------------
 * Work-queue Run() – publish events
 * -------------------------------------------------------------------------*/
void EC11RotaryEncoder::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	const hrt_abstime now = hrt_absolute_time();
	bool need_publish = false;

	rotary_encoder_event_s event{};
	event.timestamp  = now;
	event.position   = _position;

	/* --- Encoder rotation --- */
	if (_encoder_changed) {
		_encoder_changed = false;
		event.delta      = _delta;
		_delta           = 0;
		need_publish     = true;
	}

	/* --- PUSH button --- */
	if (_push_changed) {
		_push_changed = false;
		bool new_state = !stm32_gpioread(_param_gpio_push.get()); // active-low
		if (debounce_button(_push_last_change, _push_last_state, new_state, now)) {
			event.button_push   = new_state;
			event.push_rising   =  new_state;
			event.push_falling  = !new_state;
			need_publish        = true;
		}
	}

	/* --- K0 button --- */
	if (_k0_changed) {
		_k0_changed = false;
		bool new_state = !stm32_gpioread(_param_gpio_k0.get());   // active-low
		if (debounce_button(_k0_last_change, _k0_last_state, new_state, now)) {
			event.button_k0   = new_state;
			event.k0_rising   =  new_state;
			event.k0_falling  = !new_state;
			need_publish      = true;
		}
	}

	if (need_publish) {
		if (event.button_k0) {
			_position = 0;   // K0 resets the cumulative encoder position
		}

		/* Publish the raw encoder event snapshot. The display is owned by
		 * servo_test, which decides what to render. */
		event.position = _position;
		_rotary_encoder_event_pub.publish(event);
	}
}

/* -------------------------------------------------------------------------
 * Status output
 * -------------------------------------------------------------------------*/
int EC11RotaryEncoder::print_status()
{
	PX4_INFO("EC11 Rotary Encoder");
	PX4_INFO("  Position : %" PRId32, _position);
	PX4_INFO("  PUSH btn : %s", _push_last_state ? "pressed" : "released");
	PX4_INFO("  K0   btn : %s", _k0_last_state   ? "pressed" : "released");
	PX4_INFO("  GPIO A   : 0x%08X", (unsigned int)_param_gpio_a.get());
	PX4_INFO("  GPIO B   : 0x%08X", (unsigned int)_param_gpio_b.get());
	PX4_INFO("  GPIO PUSH: 0x%08X", (unsigned int)_param_gpio_push.get());
	PX4_INFO("  GPIO K0  : 0x%08X", (unsigned int)_param_gpio_k0.get());
	PX4_INFO("  Debounce : %" PRId32 " µs", _param_debounce_us.get());

	return 0;
}

/* -------------------------------------------------------------------------
 * Module entry point
 * -------------------------------------------------------------------------*/
extern "C" __EXPORT int ec11_rotary_encoder_main(int argc, char *argv[])
{
	return EC11RotaryEncoder::main(argc, argv);
}
