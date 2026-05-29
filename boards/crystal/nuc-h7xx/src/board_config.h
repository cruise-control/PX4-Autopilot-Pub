/****************************************************************************
 *
 *   Copyright (c) 2024 Crystal / Bogglingtech. All rights reserved.
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
 * @file board_config.h
 *
 * Crystal NUC-H7xx (Nucleo-H753ZI) board-specific definitions.
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include <stm32_gpio.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/

/* LEDs *******************************************************************************************/
/* Nucleo-H753ZI user LEDs — all active HIGH, push-pull
 *
 *   LD1 (Green)  : PB0
 *   LD2 (Yellow) : PE1
 *   LD3 (Red)    : PB14
 */

#define GPIO_LED_GREEN   /* PB0  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN0)
#define GPIO_LED_YELLOW  /* PE1  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN1)
#define GPIO_LED_RED     /* PB14 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN14)

#define BOARD_HAS_CONTROL_STATUS_LEDS    1
#define BOARD_OVERLOAD_LED               LED_RED
#define BOARD_ARMED_STATE_LED            LED_BLUE  /* logical BLUE → physical YELLOW (see led.c) */


/**
 * Default GPIO pin numbers.  Override at compile-time via Kconfig /
 * cmake variables, or at runtime via module parameters.
 *
 * These correspond to the board's SPI/GPIO breakout, wired as follows:
 *   A    – Encoder phase A  (quadrature)
 *   B    – Encoder phase B  (quadrature)
 *   PUSH – Encoder shaft press button
 *   K0   – Independent back/menu button
 *
 * All inputs are active-LOW with internal pull-ups enabled.
 */
#define EC11_GPIO_A     (GPIO_INPUT | GPIO_PULLUP | GPIO_EXTI | GPIO_PORTA | GPIO_PIN0)
#define EC11_GPIO_B     (GPIO_INPUT | GPIO_PULLUP | GPIO_EXTI | GPIO_PORTA | GPIO_PIN1)
#define EC11_GPIO_PUSH_BTN   /* PA02 */ (GPIO_INPUT|GPIO_PULLUP|GPIO_EXTI|GPIO_PORTA|GPIO_PIN2)
#define EC11_GPIO_K0_BTN     /* PA03 */ (GPIO_INPUT|GPIO_PULLUP|GPIO_EXTI|GPIO_PORTA|GPIO_PIN3)


// #define EC11_GPIO_K0    (GPIO_INPUT | GPIO_PULLUP | GPIO_EXTI | GPIO_PORTA | GPIO_PIN3)



/* USB OTG FS *************************************************************************************/
/* PA9 = VBUS sensing (optional; pulled low when USB not connected via ST-LINK power path) */

#define GPIO_OTGFS_VBUS  /* PA9 */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_100MHz|GPIO_PORTA|GPIO_PIN9)

/* ADC channels ***********************************************************************************/
/* Minimal ADC setup — ADC1 available for future use on CN10 */

#define ADC1_CH(n)                  (n)

/* No ADC pins used by default; uncomment to add */
/* #define PX4_ADC_GPIO \ */
/*     GPIO_ADC12_INP3    PA3 ADC1_INP15 on Arduino A0 */

#define ADC_CHANNELS 0

/* High-resolution timer **************************************************************************/
/* TIM8 on APB2 (240 MHz input) — used exclusively by HRT driver, not managed by NuttX */

#define HRT_TIMER               8  /* TIM8 */
#define HRT_TIMER_CHANNEL       3  /* CC3 output compare, no GPIO needed */

/* PWM IO timer configuration *********************************************************************/
/* TIM1 on APB2, channels on Morpho connector (CN10):
 *   CH1 : PE9
 *   CH2 : PE11
 *   CH3 : PE13
 *   CH4 : PE14
 */

#define DIRECT_PWM_OUTPUT_CHANNELS  4
#define BOARD_NUM_IO_TIMERS         1

/* Power / battery monitoring *********************************************************************/
/* No external power management IC on Nucleo — disable brick monitoring entirely. */

#define BOARD_NUMBER_BRICKS      0
#define BOARD_ADC_USB_CONNECTED  (px4_arch_gpioread(GPIO_OTGFS_VBUS))
#define BOARD_ADC_SERVO_VALID    (1)

/* DMA pool ***************************************************************************************/

#define BOARD_DMA_ALLOC_POOL_SIZE 2048

/* Console ring buffer for dmesg ******************************************************************/

#define BOARD_ENABLE_CONSOLE_BUFFER

/* Board has board_on_reset interface */

#define BOARD_HAS_ON_RESET 1

/* GPIO init list *********************************************************************************/
/* Initialise VBUS sense pin at startup; LEDs are handled by led_init(). */

#define PX4_GPIO_INIT_LIST { \
		GPIO_OTGFS_VBUS, \
	}

__BEGIN_DECLS

#ifndef __ASSEMBLY__

extern void stm32_usbinitialize(void);
extern void board_peripheral_reset(int ms);
extern void stm32_spiinitialize(void);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
