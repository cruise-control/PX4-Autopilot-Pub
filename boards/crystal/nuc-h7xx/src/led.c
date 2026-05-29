/****************************************************************************
 *
 *   Copyright (c) 2024 Crystal / Bogglingtech. All rights reserved.
 *   Based on PX4 Development Team led.c.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file led.c
 *
 * Crystal NUC-H7xx (Nucleo-H753ZI) LED driver.
 *
 * All three Nucleo user LEDs are ACTIVE HIGH, push-pull:
 *   LD1 Green  : PB0
 *   LD2 Yellow : PE1
 *   LD3 Red    : PB14
 */

#include <px4_platform_common/px4_config.h>

#include <stdbool.h>

#include "chip.h"
#include "stm32_gpio.h"
#include "board_config.h"

#include <nuttx/board.h>
#include <arch/board/board.h>

__BEGIN_DECLS
extern void led_init(void);
extern void led_on(int led);
extern void led_off(int led);
extern void led_toggle(int led);
__END_DECLS

/* PX4 platform LED indices (drv_board_led.h):
 *   LED_BLUE   = 0
 *   LED_RED    = 1   (also LED_AMBER)
 *   LED_SAFETY = 2
 *   LED_GREEN  = 3
 *
 * Physical LED mapping for nuc-h7xx (active HIGH, no inversion needed):
 *   [0] LED_BLUE   → Yellow LD2 (status indicator, closest substitute)
 *   [1] LED_RED    → Red    LD3 (error / overload)
 *   [2] LED_SAFETY → (none — no safety LED on Nucleo)
 *   [3] LED_GREEN  → Green  LD1 (normal running)
 */

#ifdef CONFIG_ARCH_LEDS
static bool nuttx_owns_leds = true;
/* Maps PX4 LED index → board g_ledmap index:
 *   LED_BLUE  (0) → 1 (YELLOW)
 *   LED_RED   (1) → 2 (RED)
 *   LED_SAFETY(2) → 3 (no-op slot, g_ledmap[3]=0)
 *   LED_GREEN (3) → 0 (GREEN)
 */
static const uint8_t xlatpx4[] = {1, 2, 3, 0};
#  define xlat(p) xlatpx4[(p)]

static uint32_t g_ledmap[] = {
	GPIO_LED_GREEN,   /* [0] BOARD_LED_GREEN  */
	GPIO_LED_YELLOW,  /* [1] BOARD_LED_YELLOW */
	GPIO_LED_RED,     /* [2] BOARD_LED_RED    */
	0,                /* [3] LED_SAFETY no-op (no dedicated safety LED) */
};

#else

#  define xlat(p) (p)
static uint32_t g_ledmap[] = {
	GPIO_LED_YELLOW,  /* [0] LED_BLUE   → Yellow */
	GPIO_LED_RED,     /* [1] LED_RED    → Red    */
	0,                /* [2] LED_SAFETY → none   */
	GPIO_LED_GREEN,   /* [3] LED_GREEN  → Green  */
};

#endif /* CONFIG_ARCH_LEDS */

__EXPORT void led_init(void)
{
	for (size_t l = 0; l < sizeof(g_ledmap) / sizeof(g_ledmap[0]); l++) {
		if (g_ledmap[l] != 0) {
			stm32_configgpio(g_ledmap[l]);
		}
	}
}

/* Drive HIGH to illuminate (active-HIGH LEDs — no inversion). */
static void phy_set_led(int led, bool state)
{
	if ((size_t)led < sizeof(g_ledmap) / sizeof(g_ledmap[0]) && g_ledmap[led] != 0) {
		stm32_gpiowrite(g_ledmap[led], state);
	}
}

static bool phy_get_led(int led)
{
	if ((size_t)led < sizeof(g_ledmap) / sizeof(g_ledmap[0]) && g_ledmap[led] != 0) {
		return stm32_gpioread(g_ledmap[led]);
	}

	return false;
}

__EXPORT void led_on(int led)
{
	phy_set_led(xlat(led), true);
}

__EXPORT void led_off(int led)
{
	phy_set_led(xlat(led), false);
}

__EXPORT void led_toggle(int led)
{
	int idx = xlat(led);
	phy_set_led(idx, !phy_get_led(idx));
}

#ifdef CONFIG_ARCH_LEDS

void board_autoled_initialize(void)
{
	led_init();
}

void board_autoled_on(int led)
{
	if (!nuttx_owns_leds) {
		return;
	}

	switch (led) {
	case LED_HEAPALLOCATE:
		phy_set_led(BOARD_LED_GREEN, true);
		break;

	case LED_IRQSENABLED:
		phy_set_led(BOARD_LED_GREEN, false);
		phy_set_led(BOARD_LED_YELLOW, true);
		break;

	case LED_STACKCREATED:
		phy_set_led(BOARD_LED_GREEN, true);
		phy_set_led(BOARD_LED_YELLOW, true);
		break;

	case LED_INIRQ:
		phy_set_led(BOARD_LED_YELLOW, true);
		break;

	case LED_SIGNAL:
		phy_set_led(BOARD_LED_GREEN, true);
		break;

	case LED_ASSERTION:
		phy_set_led(BOARD_LED_YELLOW, true);
		phy_set_led(BOARD_LED_RED, true);
		break;

	case LED_PANIC:
		phy_set_led(BOARD_LED_RED, true);
		break;

	case LED_IDLE:
		phy_set_led(BOARD_LED_RED, true);
		break;

	default:
		break;
	}
}

void board_autoled_off(int led)
{
	if (!nuttx_owns_leds) {
		return;
	}

	switch (led) {
	case LED_IRQSENABLED:
		phy_set_led(BOARD_LED_YELLOW, false);
		break;

	case LED_INIRQ:
		phy_set_led(BOARD_LED_YELLOW, false);
		break;

	case LED_SIGNAL:
		phy_set_led(BOARD_LED_GREEN, false);
		break;

	case LED_ASSERTION:
		phy_set_led(BOARD_LED_YELLOW, false);
		phy_set_led(BOARD_LED_RED, false);
		break;

	case LED_PANIC:
		phy_set_led(BOARD_LED_RED, false);
		break;

	case LED_IDLE:
		phy_set_led(BOARD_LED_RED, false);
		break;

	default:
		break;
	}
}

#endif /* CONFIG_ARCH_LEDS */
