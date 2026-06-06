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
#define FLASH_BASED_PARAMS
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


/* HMI pin map — EC11 rotary encoder + ST7789 SPI display *****************************************
 *
 * The ec11_rotary_encoder and st7789_display drivers read their pins from
 * runtime parameters (EC11_GPIO_*, ST7789_GPIO_*) rather than these macros.
 * The board defaults that wire them up live in init/rc.board_defaults; the
 * macros below are the authoritative, human-readable definition of the wiring
 * and the comment shows the matching GPIO config word used there.
 *
 * Every pin below is exposed on the Nucleo-H753ZI Arduino (Uno R3) header so
 * the board can drive a standard Arduino-form display/encoder HAT. The Arduino
 * label (Ax/Dx) is shown next to each STM32 pin.
 *
 * EC11 rotary encoder — inputs, active-LOW with internal pull-ups + EXTI.
 * Pin numbers differ (3/0/1/2) so each maps to a distinct EXTI line:
 *   A    – phase A  (quadrature)  A0  PA3   0x00010103  (EXTI3)
 *   B    – phase B  (quadrature)  A1  PC0   0x00010120  (EXTI0)
 *   PUSH – shaft press button     A3  PB1   0x00010111  (EXTI1)
 *   K0   – back/menu button       A4  PC2   0x00010122  (EXTI2)
 *
 * ST7789 display — SPI1 (SCK PA5/D13, MISO PA6/D12, MOSI PB5/D11) plus control
 * outputs (push-pull, 50 MHz):
 *   CS   – chip-select (idle high) D10 PD14  0x0004093E  (software-driven, see spi.cpp)
 *   DC   – data/command            D9  PD15  0x0004083F
 *   RES  – reset                   D8  PF3   0x00040853
 *   BLT  – backlight               D7  PG12  0x0004086C
 */
#define EC11_GPIO_IO_A     /* A0/PA3  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_EXTI|GPIO_PORTA|GPIO_PIN3)
#define EC11_GPIO_IO_B     /* A1/PC0  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_EXTI|GPIO_PORTC|GPIO_PIN0)
#define EC11_GPIO_IO_USR_PUSH  /* A3/PB1  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_EXTI|GPIO_PORTB|GPIO_PIN1)
#define EC11_GPIO_IO_USR_K0    /* A4/PC2  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_EXTI|GPIO_PORTC|GPIO_PIN2)

#define ST7789_GPIO_IO_CS  /* D10/PD14 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN14)
#define ST7789_GPIO_IO_DC  /* D9/PD15  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN15)
#define ST7789_GPIO_IO_RES /* D8/PF3   */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTF|GPIO_PIN3)
#define ST7789_GPIO_IO_BLk /* D7/PG12  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN12)


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
/* 8 PWM outputs across three timers (see timer_config.cpp).
 * Arduino labels in brackets — TIM4/TIM2 are the second group added for 8-ch support.
 *
 *   TIM1  CH1 PE9 (D6)  CH2 PE11 (D5)  CH3 PE13 (D3)  CH4 PE14 (D4)  — Arduino subset
 *   TIM4  CH1 PB6 (D1)  CH2 PB7  (D0)                                — Arduino subset
 *   TIM2  CH3 PB10 (CN10-32/D36)  CH4 PB11 (CN10-34/D35)            — ST Zio extension
 *
 * Only two Arduino-subset pins (PB6/PB7) have a free timer output, so the last
 * two channels use PB10/PB11. Those are NOT on the Arduino subset but ARE on the
 * ST Zio connector (CN10 pins 32/34), so all 8 outputs reach a Zio-form HAT.
 */

#define DIRECT_PWM_OUTPUT_CHANNELS  8
#define BOARD_NUM_IO_TIMERS         3

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

#define UAVCAN_NUM_IFACES_RUNTIME 1

/* GPIO init list *********************************************************************************/
/* Initialise VBUS sense pin at startup; LEDs are handled by led_init(). */

#define PX4_GPIO_INIT_LIST { \
		GPIO_OTGFS_VBUS, \
		GPIO_CAN1_TX, \
		GPIO_CAN1_RX, \
	}

__BEGIN_DECLS

#ifndef __ASSEMBLY__

extern void stm32_usbinitialize(void);
extern void board_peripheral_reset(int ms);
extern void stm32_spiinitialize(void);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
