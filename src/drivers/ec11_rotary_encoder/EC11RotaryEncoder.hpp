/****************************************************************************
 * drivers/ec11_rotary_encoder/EC11RotaryEncoder.hpp
 *
 * PX4 driver for the EC11 rotary encoder with push-button (PUSH pin) and
 * independent back/menu button (K0 pin) as found on the combined
 * 2.4" TFT + EC11 board (ST7789 / SPI display, B0GW29KWS9).
 *
 * Interface: GPIO interrupts on A, B, PUSH, and K0 pins.
 * Publishes:  rotary_encoder_event uORB topic.
 *
 * BSD 3-Clause License  –  see LICENSE file for details.
 ****************************************************************************/

#pragma once

#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/rotary_encoder_event.h>
#include <drivers/drv_hrt.h>
#include <lib/parameters/param.h>

#include <nuttx/ioexpander/gpio.h>   // NuttX GPIO API

using namespace time_literals;

/** Software debounce period for buttons [µs] */
#define EC11_BTN_DEBOUNCE_US  5000ULL

/** Maximum encoder step rate before saturation [steps/s] */
#define EC11_MAX_RATE   1000

class EC11RotaryEncoder : public ModuleBase<EC11RotaryEncoder>,
                          public ModuleParams,
                          public px4::ScheduledWorkItem
{
public:
	EC11RotaryEncoder();
	~EC11RotaryEncoder() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	/** ModuleBase interface */
	bool init();

	/** Print current driver state to console */
	int print_status() override;

private:
	/* ----- ScheduledWorkItem callback ----- */
	void Run() override;

	/* ----- GPIO interrupt handlers (static trampolines) ----- */
	static int encoder_a_isr(int irq, void *context, void *arg);
	static int encoder_b_isr(int irq, void *context, void *arg);
	static int push_isr    (int irq, void *context, void *arg);
	static int k0_isr      (int irq, void *context, void *arg);

	/* ----- Internal helpers ----- */
	void process_encoder_step(bool a_level, bool b_level);
	bool debounce_button(hrt_abstime &last_change, bool &last_state,
	                     bool new_state, hrt_abstime now);

	/* ----- uORB publication ----- */
	uORB::PublicationMulti<rotary_encoder_event_s> _pub{ORB_ID(rotary_encoder_event)};

	/* ----- Encoder state (accessed from ISR context, so volatile) ----- */
	volatile int32_t  _position{0};
	volatile int8_t   _delta{0};
	volatile uint8_t  _ab_state{0};   ///< Previous AB state for quadrature decode

	/* ----- Button state ----- */
	hrt_abstime _push_last_change{0};
	hrt_abstime _k0_last_change{0};
	bool        _push_last_state{true};   ///< Active low → idle=true
	bool        _k0_last_state{true};

	/* ----- Pending event flags set from ISR, consumed in Run() ----- */
	volatile bool _encoder_changed{false};
	volatile bool _push_changed{false};
	volatile bool _k0_changed{false};

	/* ----- Parameters ----- */
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::EC11_GPIO_PIN_A>)    _param_gpio_a,
		(ParamInt<px4::params::EC11_GPIO_PIN_B>)    _param_gpio_b,
		(ParamInt<px4::params::EC11_GPIO_PUSH>)     _param_gpio_push,
		(ParamInt<px4::params::EC11_GPIO_K0>)       _param_gpio_k0,
		(ParamInt<px4::params::EC11_DEBOUNCE_US>)   _param_debounce_us
	)
};
