/****************************************************************************
 * drivers/st7789_display/ST7789Display.hpp
 *
 * PX4 driver for the ST7789-based 2.4" TFT SPI display module.
 * Target hardware: 2.4" 240×320 SPI TFT as found on the EC11 combo board
 * (Amazon B0GW29KWS9).
 *
 * Pin mapping (SPI + control GPIOs):
 *   SCL  – SPI SCLK
 *   SDA  – SPI MOSI  (MISO not used – display is write-only)
 *   RES  – Active-low hardware reset  (GPIO output)
 *   DC   – Data/Command select        (GPIO output, high=data low=cmd)
 *   CS   – Chip select, active low    (GPIO output or SPI CS)
 *   BLK  – Backlight PWM / on-off     (GPIO/PWM output, optional)
 *
 * Display layout:
 * ┌─────────────────────────────────┐
 * │  STATUS BAR  (top 36 px)        │  ← status_text, coloured background
 * ├─────────────────────────────────┤
 * │                                 │
 * │      LARGE NUMERIC VALUE        │  ← numeric_value centred in large font
 * │           1234.56               │
 * │            [units]              │
 * │                                 │
 * └─────────────────────────────────┘
 *
 * Renders using a minimal software framebuffer / direct-draw approach
 * (no OS framebuffer required).  Bitmap font is embedded for standalone
 * operation (no external file-system dependency).
 *
 * BSD 3-Clause License – see LICENSE for details.
 ****************************************************************************/

#pragma once

#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/display_command.h>
#include <drivers/drv_hrt.h>
#include <lib/parameters/param.h>

#include <nuttx/spi/spi.h>
#include <stm32_gpio.h>

using namespace time_literals;

/* ---- Display geometry -------------------------------------------------- */
#define ST7789_WIDTH       240u
#define ST7789_HEIGHT      320u
#define ST7789_STATUS_H     36u   ///< Height of status bar [pixels]
#define ST7789_NUMERIC_Y   (ST7789_STATUS_H + 20u)

/* ---- ST7789 command bytes ---------------------------------------------- */
#define ST7789_CMD_NOP       0x00
#define ST7789_CMD_SWRESET   0x01
#define ST7789_CMD_SLPOUT    0x11
#define ST7789_CMD_NORON     0x13
#define ST7789_CMD_INVOFF    0x20
#define ST7789_CMD_INVON     0x21
#define ST7789_CMD_DISPON    0x29
#define ST7789_CMD_CASET     0x2A
#define ST7789_CMD_RASET     0x2B
#define ST7789_CMD_RAMWR     0x2C
#define ST7789_CMD_MADCTL    0x36
#define ST7789_CMD_COLMOD    0x3A
#define ST7789_CMD_PORCTRL   0xB2
#define ST7789_CMD_GCTRL     0xB7
#define ST7789_CMD_VCOMS     0xBB
#define ST7789_CMD_LCMCTRL   0xC0
#define ST7789_CMD_VDVVRHEN  0xC2
#define ST7789_CMD_VRHS      0xC3
#define ST7789_CMD_VDVS      0xC4
#define ST7789_CMD_FRCTRL2   0xC6
#define ST7789_CMD_PWCTRL1   0xD0
#define ST7789_CMD_PVGAMCTRL 0xE0
#define ST7789_CMD_NVGAMCTRL 0xE1

/* ---- RGB565 colour helpers ---------------------------------------------- */
#define RGB565(r,g,b) ((uint16_t)(((r) & 0xF8u) << 8u | ((g) & 0xFCu) << 3u | ((b) >> 3u)))

#define COLOR_BLACK   RGB565(  0,   0,   0)
#define COLOR_WHITE   RGB565(255, 255, 255)
#define COLOR_GREEN   RGB565(  0, 220,   0)
#define COLOR_YELLOW  RGB565(255, 220,   0)
#define COLOR_RED     RGB565(220,   0,   0)
#define COLOR_BLUE    RGB565(  0,  80, 220)
#define COLOR_DGRAY   RGB565( 32,  32,  32)
#define COLOR_LGRAY   RGB565(160, 160, 160)

class ST7789Display : public ModuleBase<ST7789Display>,
                      public ModuleParams,
                      public px4::ScheduledWorkItem
{
public:
	ST7789Display();
	~ST7789Display() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	/* ----- Work loop ------------------------------------------------------ */
	void Run() override;

	/* ----- Hardware init -------------------------------------------------- */
	bool hw_init();
	void hardware_reset();
	void send_cmd(uint8_t cmd);
	void send_data(const uint8_t *data, size_t len);
	void send_data_u16(uint16_t word);
	void set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
	void fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);

	/* ----- Rendering ------------------------------------------------------ */
	void render_status_bar(const char *text, uint8_t color_code);
	void render_numeric(float value, uint8_t decimal_places,
	                    const char *units);
	void render_full(const display_command_s &cmd);

	/* ----- Bitmap font helpers -------------------------------------------- */
	void draw_char_large(uint16_t x, uint16_t y, char c,
	                     uint16_t fg, uint16_t bg);
	void draw_string_large(uint16_t x, uint16_t y, const char *str,
	                       uint16_t fg, uint16_t bg);
	uint16_t string_pixel_width_large(const char *str);

	void draw_char_small(uint16_t x, uint16_t y, char c,
	                     uint16_t fg, uint16_t bg);
	void draw_string_small(uint16_t x, uint16_t y, const char *str,
	                       uint16_t fg, uint16_t bg);

	/* ----- SPI bus -------------------------------------------------------- */
	struct spi_dev_s *_spi{nullptr};
	uint32_t          _spi_bus{1};
	uint32_t          _spi_freq{40000000};  ///< 40 MHz – ST7789 max

	/* ----- Active drawing geometry (swapped in landscape) ----------------- */
	uint16_t _w{ST7789_WIDTH};
	uint16_t _h{ST7789_HEIGHT};

	/* ----- GPIO config words (set from params) ----------------------------- */
	uint32_t _gpio_res{0};
	uint32_t _gpio_dc{0};
	uint32_t _gpio_cs{0};
	uint32_t _gpio_blk{0};

	/* ----- Display state cache -------------------------------------------- */
	float    _last_value{0.0f};
	char     _last_status[32]{};
	uint8_t  _last_status_color{0xFF};   ///< Force first draw
	bool     _initialized{false};

	/* ----- uORB ----------------------------------------------------------- */
	uORB::Subscription _cmd_sub{ORB_ID(display_command)};

	/* ----- Parameters ----------------------------------------------------- */
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::ST7789_SPI_BUS>)      _param_spi_bus,
		(ParamInt<px4::params::ST7789_SPI_FREQ>)     _param_spi_freq,
		(ParamInt<px4::params::ST7789_GPIO_RES>)     _param_gpio_res,
		(ParamInt<px4::params::ST7789_GPIO_DC>)      _param_gpio_dc,
		(ParamInt<px4::params::ST7789_GPIO_CS>)      _param_gpio_cs,
		(ParamInt<px4::params::ST7789_GPIO_BLK>)     _param_gpio_blk,
		(ParamInt<px4::params::ST7789_ROTATE>)       _param_rotate
	)
};
