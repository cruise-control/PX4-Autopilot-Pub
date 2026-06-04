/****************************************************************************
 * drivers/st7789_display/ST7789Display.cpp
 *
 * PX4 driver for the ST7789-based 2.4" 240×320 SPI TFT display.
 *
 * Subscribes to `display_command` uORB topic and renders:
 *   • Top status bar  – coloured background + status message text
 *   • Large numeric   – centred large-font value + units
 *
 * SPI is operated in Mode 3 (CPOL=1, CPHA=1), MSB first, up to 40 MHz.
 * The DC (data/command) pin is toggled around every SPI transaction.
 *
 * BSD 3-Clause License – see LICENSE for details.
 ****************************************************************************/

#include "ST7789Display.hpp"
#include "font_data.hpp"

#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include <stm32_gpio.h>
#include <stm32_spi.h>

/* --------------------------------------------------------------------------
 * Construction / destruction
 * --------------------------------------------------------------------------*/
ST7789Display::ST7789Display() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

ST7789Display::~ST7789Display()
{
	ScheduleClear();

	/* Backlight off */
	if (_gpio_blk) {
		stm32_gpiowrite(_gpio_blk, false);
	}

	if (_spi) {
		SPI_LOCK(_spi, false);
	}
}

/* --------------------------------------------------------------------------
 * ModuleBase entry points
 * --------------------------------------------------------------------------*/
int ST7789Display::task_spawn(int argc, char *argv[])
{
	ST7789Display *obj = new ST7789Display();

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

int ST7789Display::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ST7789Display::print_usage(const char *reason)
{
	if (reason) { PX4_WARN("%s\n", reason); }

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver for the ST7789-based 2.4" 240×320 SPI TFT display module (B0GW29KWS9).

Subscribes to `display_command` uORB topic and renders a two-zone layout:
  • Top status bar (coloured background + message text)
  • Large centred numeric value with units

### Parameters
ST7789_SPI_BUS       – SPI bus number (default 1)
ST7789_SPI_FREQ      – SPI clock frequency in kHz (default 40000)
ST7789_GPIO_RES      – GPIO config word for RES (reset) pin
ST7789_GPIO_DC       – GPIO config word for DC (data/command) pin
ST7789_GPIO_CS       – GPIO config word for CS (chip-select) pin
ST7789_GPIO_BLK      – GPIO config word for BLK (backlight) pin
ST7789_ROTATE        – Display rotation: 0=portrait 1=landscape
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("st7789_display", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

/* --------------------------------------------------------------------------
 * Initialisation
 * --------------------------------------------------------------------------*/
bool ST7789Display::init()
{
	updateParams();

	_spi_bus   = static_cast<uint32_t>(_param_spi_bus.get());
	_spi_freq  = static_cast<uint32_t>(_param_spi_freq.get()) * 1000u;
	_gpio_res  = static_cast<uint32_t>(_param_gpio_res.get());
	_gpio_dc   = static_cast<uint32_t>(_param_gpio_dc.get());
	_gpio_cs   = static_cast<uint32_t>(_param_gpio_cs.get());
	_gpio_blk  = static_cast<uint32_t>(_param_gpio_blk.get());

	/* Active drawing geometry: landscape (rotate==1) swaps width/height so the
	 * rest of the renderer addresses the panel with the correct dimensions. */
	if (_param_rotate.get() == 1) {
		_w = ST7789_HEIGHT;   // 320
		_h = ST7789_WIDTH;    // 240
	} else {
		_w = ST7789_WIDTH;    // 240
		_h = ST7789_HEIGHT;   // 320
	}

	/* Configure control GPIOs as outputs */
	stm32_configgpio(_gpio_res);
	stm32_configgpio(_gpio_dc);
	stm32_configgpio(_gpio_cs);
	if (_gpio_blk) { stm32_configgpio(_gpio_blk); }

	/* Get SPI bus handle */
	_spi = stm32_spibus_initialize(static_cast<int>(_spi_bus));

	if (!_spi) {
		PX4_ERR("SPI bus %ld init failed", _spi_bus);
		return false;
	}

	SPI_SETMODE(_spi, SPIDEV_MODE3);
	SPI_SETBITS(_spi, 8);
	SPI_SETFREQUENCY(_spi, _spi_freq);

	if (!hw_init()) {
		PX4_ERR("ST7789 hardware init failed");
		return false;
	}

	_initialized = true;

	/* Draw initial blank screen */
	fill_rect(0, 0, _w, _h, COLOR_BLACK);
	render_status_bar("INIT", 0);

	/* Subscribe and schedule */
	_cmd_sub.subscribe();
	ScheduleOnInterval(33_ms);   // ~30 Hz refresh

	PX4_INFO("ST7789 display driver started  SPI%ld @ %ld kHz", _spi_bus, _spi_freq/1000);
	return true;
}

/* --------------------------------------------------------------------------
 * ST7789 hardware initialisation sequence
 * --------------------------------------------------------------------------*/
bool ST7789Display::hw_init()
{
	/* Assert chip-select and pull reset low */
	stm32_gpiowrite(_gpio_cs, false);
	hardware_reset();

	/* Initialisation sequence based on ST7789 datasheet / common Arduino libs */
	send_cmd(ST7789_CMD_SWRESET);
	px4_usleep(150000);

	send_cmd(ST7789_CMD_SLPOUT);
	px4_usleep(10000);

	/* Colour format: 16-bit RGB565 */
	send_cmd(ST7789_CMD_COLMOD);
	uint8_t colmod = 0x55;
	send_data(&colmod, 1);
	px4_usleep(10000);

	/* Memory access control – orientation + colour order.
	 * Bit layout: MY MX MV ML RGB MH . . (0x08 = BGR).
	 *   landscape (320x240) = MV|MX = 0x60 ; portrait (240x320) = 0x00.
	 * The BGR bit (0x08) is set because this panel wires the sub-pixels B-G-R;
	 * leaving it clear made red/blue swap (yellow rendered red). */
	send_cmd(ST7789_CMD_MADCTL);
	uint8_t madctl = (_param_rotate.get() == 1) ? 0x68 : 0x08;
	send_data(&madctl, 1);

	/* Porch control */
	send_cmd(ST7789_CMD_PORCTRL);
	{
		uint8_t d[] = {0x0C, 0x0C, 0x00, 0x33, 0x33};
		send_data(d, sizeof(d));
	}

	send_cmd(ST7789_CMD_GCTRL);
	{ uint8_t d = 0x35; send_data(&d, 1); }

	send_cmd(ST7789_CMD_VCOMS);
	{ uint8_t d = 0x28; send_data(&d, 1); }

	send_cmd(ST7789_CMD_LCMCTRL);
	{ uint8_t d = 0x0C; send_data(&d, 1); }

	send_cmd(ST7789_CMD_VDVVRHEN);
	{ uint8_t d[] = {0x01, 0xFF}; send_data(d, 2); }

	send_cmd(ST7789_CMD_VRHS);
	{ uint8_t d = 0x10; send_data(&d, 1); }

	send_cmd(ST7789_CMD_VDVS);
	{ uint8_t d = 0x20; send_data(&d, 1); }

	send_cmd(ST7789_CMD_FRCTRL2);
	{ uint8_t d = 0x0F; send_data(&d, 1); }

	send_cmd(ST7789_CMD_PWCTRL1);
	{ uint8_t d[] = {0xA4, 0xA1}; send_data(d, 2); }

	/* Gamma curves (positive / negative) */
	send_cmd(ST7789_CMD_PVGAMCTRL);
	{
		uint8_t d[] = {0xD0,0x00,0x02,0x07,0x0A,0x28,0x32,0x44,
		               0x42,0x06,0x0E,0x12,0x14,0x17};
		send_data(d, sizeof(d));
	}

	send_cmd(ST7789_CMD_NVGAMCTRL);
	{
		uint8_t d[] = {0xD0,0x00,0x02,0x07,0x0A,0x28,0x31,0x54,
		               0x47,0x0E,0x1C,0x17,0x1B,0x1E};
		send_data(d, sizeof(d));
	}

	/* This panel is non-inverting: forcing INVON made every colour come out
	 * inverted (white text rendered black, black background rendered white /
	 * "spurious"). Use INVOFF so the RGB565 values map straight through. */
	send_cmd(ST7789_CMD_INVOFF);
	send_cmd(ST7789_CMD_NORON);
	px4_usleep(10000);

	send_cmd(ST7789_CMD_DISPON);
	px4_usleep(10000);

	/* Backlight on */
	if (_gpio_blk) {
		stm32_gpiowrite(_gpio_blk, true);
	}

	stm32_gpiowrite(_gpio_cs, true);
	return true;
}

/* --------------------------------------------------------------------------
 * Hardware reset (toggle RES pin)
 * --------------------------------------------------------------------------*/
void ST7789Display::hardware_reset()
{
	stm32_gpiowrite(_gpio_res, true);
	px4_usleep(10000);
	stm32_gpiowrite(_gpio_res, false);
	px4_usleep(10000);
	stm32_gpiowrite(_gpio_res, true);
	px4_usleep(120000);
}

/* --------------------------------------------------------------------------
 * Low-level SPI helpers
 * --------------------------------------------------------------------------*/
void ST7789Display::send_cmd(uint8_t cmd)
{
	stm32_gpiowrite(_gpio_dc, false);  // Command mode
	stm32_gpiowrite(_gpio_cs, false);
	SPI_SEND(_spi, cmd);
	stm32_gpiowrite(_gpio_cs, true);
}

void ST7789Display::send_data(const uint8_t *data, size_t len)
{
	stm32_gpiowrite(_gpio_dc, true);   // Data mode
	stm32_gpiowrite(_gpio_cs, false);
	SPI_SNDBLOCK(_spi, data, len);
	stm32_gpiowrite(_gpio_cs, true);
}

void ST7789Display::send_data_u16(uint16_t word)
{
	uint8_t buf[2] = { static_cast<uint8_t>(word >> 8),
	                   static_cast<uint8_t>(word & 0xFF) };
	send_data(buf, 2);
}

/* --------------------------------------------------------------------------
 * Set active window (CASET / RASET)
 * --------------------------------------------------------------------------*/
void ST7789Display::set_window(uint16_t x0, uint16_t y0,
                               uint16_t x1, uint16_t y1)
{
	send_cmd(ST7789_CMD_CASET);
	{
		uint8_t d[] = { static_cast<uint8_t>(x0 >> 8), static_cast<uint8_t>(x0),
		                static_cast<uint8_t>(x1 >> 8), static_cast<uint8_t>(x1) };
		send_data(d, 4);
	}

	send_cmd(ST7789_CMD_RASET);
	{
		uint8_t d[] = { static_cast<uint8_t>(y0 >> 8), static_cast<uint8_t>(y0),
		                static_cast<uint8_t>(y1 >> 8), static_cast<uint8_t>(y1) };
		send_data(d, 4);
	}

	send_cmd(ST7789_CMD_RAMWR);
}

/* --------------------------------------------------------------------------
 * Bulk pixel blit: one CS-framed SPI_SNDBLOCK. With SPI DMA enabled this is a
 * single DMA transfer instead of one 2-byte PIO write per pixel. The caller
 * must have already issued set_window() (which leaves CS de-asserted), and
 * len must be <= sizeof(_blit) (== the SPI DMA buffer) to stay on the DMA path.
 * --------------------------------------------------------------------------*/
void ST7789Display::blit(const uint8_t *buf, size_t len)
{
	stm32_gpiowrite(_gpio_dc, true);    // data
	stm32_gpiowrite(_gpio_cs, false);   // re-assert CS (set_window left it high)
	SPI_SNDBLOCK(_spi, buf, len);
	stm32_gpiowrite(_gpio_cs, true);
}

/* --------------------------------------------------------------------------
 * Fill a rectangular region with a solid colour
 * --------------------------------------------------------------------------*/
void ST7789Display::fill_rect(uint16_t x, uint16_t y,
                              uint16_t w, uint16_t h, uint16_t color)
{
	if (!_spi) { return; }

	const uint32_t pixels = static_cast<uint32_t>(w) * h;
	if (pixels == 0u) { return; }

	/* Pre-expand the colour once into the shared blit buffer (up to 512 px =
	 * 1024 B = the SPI DMA buffer) and stream it out in DMA-sized chunks. */
	const uint8_t hi = static_cast<uint8_t>(color >> 8);
	const uint8_t lo = static_cast<uint8_t>(color & 0xFF);
	const uint16_t bufpx = (pixels < kBlitPixels) ? static_cast<uint16_t>(pixels) : kBlitPixels;
	for (uint16_t i = 0; i < bufpx; i++) {
		_blit[i * 2u]      = hi;
		_blit[i * 2u + 1u] = lo;
	}

	set_window(x, y, static_cast<uint16_t>(x + w - 1u),
	                 static_cast<uint16_t>(y + h - 1u));

	/* set_window() left CS de-asserted; hold it low for the whole RAMWR stream. */
	stm32_gpiowrite(_gpio_dc, true);
	stm32_gpiowrite(_gpio_cs, false);
	for (uint32_t p = 0; p < pixels; p += bufpx) {
		const uint32_t chunk = ((p + bufpx) <= pixels) ? bufpx : (pixels - p);
		SPI_SNDBLOCK(_spi, _blit, chunk * 2u);
	}
	stm32_gpiowrite(_gpio_cs, true);
}

/* --------------------------------------------------------------------------
 * Draw a single character using the small 8×16 font
 * --------------------------------------------------------------------------*/
void ST7789Display::draw_char_small(uint16_t x, uint16_t y, char c,
                                    uint16_t fg, uint16_t bg)
{
	if (c < kFont8x16FirstChar || c > 0x7E) { c = '?'; }

	const uint8_t *glyph = kFont8x16[static_cast<uint8_t>(c) - kFont8x16FirstChar];

	/* Expand the glyph into RGB565 big-endian bytes, then blit in one transfer
	 * (8x16 = 128 px = 256 B) instead of 128 individual 2-byte PIO writes. */
	const uint8_t fhi = static_cast<uint8_t>(fg >> 8), flo = static_cast<uint8_t>(fg & 0xFF);
	const uint8_t bhi = static_cast<uint8_t>(bg >> 8), blo = static_cast<uint8_t>(bg & 0xFF);
	size_t n = 0;
	for (uint16_t row = 0; row < kFont8x16GlyphH; row++) {
		uint8_t bits = glyph[row];
		for (uint8_t col = 0; col < kFont8x16GlyphW; col++) {
			if (bits & 0x80u) { _blit[n++] = fhi; _blit[n++] = flo; }
			else              { _blit[n++] = bhi; _blit[n++] = blo; }
			bits <<= 1u;
		}
	}

	set_window(x, y,
	           static_cast<uint16_t>(x + kFont8x16GlyphW - 1u),
	           static_cast<uint16_t>(y + kFont8x16GlyphH - 1u));
	blit(_blit, n);
}

void ST7789Display::draw_string_small(uint16_t x, uint16_t y, const char *str,
                                      uint16_t fg, uint16_t bg)
{
	while (*str) {
		draw_char_small(x, y, *str++, fg, bg);
		x = static_cast<uint16_t>(x + kFont8x16GlyphW);
	}
}

/* --------------------------------------------------------------------------
 * Draw a single character using the large 16×32 font
 * --------------------------------------------------------------------------*/
void ST7789Display::draw_char_large(uint16_t x, uint16_t y, char c,
                                    uint16_t fg, uint16_t bg)
{
	const int idx = large_glyph_index(c);
	const uint16_t *glyph = kFontLarge[idx];

	/* Expand the glyph into RGB565 big-endian bytes, then blit in one transfer
	 * (16x32 = 512 px = 1024 B, exactly the SPI DMA buffer) instead of 512
	 * individual 2-byte PIO writes. */
	const uint8_t fhi = static_cast<uint8_t>(fg >> 8), flo = static_cast<uint8_t>(fg & 0xFF);
	const uint8_t bhi = static_cast<uint8_t>(bg >> 8), blo = static_cast<uint8_t>(bg & 0xFF);
	size_t n = 0;
	for (uint16_t row = 0; row < kFontLargeGlyphH; row++) {
		uint16_t bits = glyph[row];
		for (uint16_t col = 0; col < kFontLargeGlyphW; col++) {
			if (bits & 0x8000u) { _blit[n++] = fhi; _blit[n++] = flo; }
			else                { _blit[n++] = bhi; _blit[n++] = blo; }
			bits = static_cast<uint16_t>(bits << 1u);
		}
	}

	set_window(x, y,
	           static_cast<uint16_t>(x + kFontLargeGlyphW - 1u),
	           static_cast<uint16_t>(y + kFontLargeGlyphH - 1u));
	blit(_blit, n);
}

void ST7789Display::draw_string_large(uint16_t x, uint16_t y, const char *str,
                                      uint16_t fg, uint16_t bg)
{
	while (*str) {
		draw_char_large(x, y, *str++, fg, bg);
		x = static_cast<uint16_t>(x + kFontLargeGlyphW);
	}
}

uint16_t ST7789Display::string_pixel_width_large(const char *str)
{
	uint16_t n = 0;
	while (*str++) { n++; }
	return static_cast<uint16_t>(n * kFontLargeGlyphW);
}

/* --------------------------------------------------------------------------
 * Render the top status bar
 * --------------------------------------------------------------------------*/
void ST7789Display::render_status_bar(const char *text, uint8_t color_code)
{
	/* Background colour */
	uint16_t bg;
	switch (color_code) {
	case 1:  bg = COLOR_GREEN;  break;
	case 2:  bg = COLOR_YELLOW; break;
	case 3:  bg = COLOR_RED;    break;
	default: bg = COLOR_BLUE;   break;
	}

	fill_rect(0, 0, _w, ST7789_STATUS_H, bg);

	/* Centred status text using small font */
	const size_t  len   = strnlen(text, 31);
	const uint16_t tw   = static_cast<uint16_t>(len * kFont8x16GlyphW);
	const uint16_t tx   = (_w  > tw) ? ((_w - tw) / 2u) : 0u;
	const uint16_t ty   = (ST7789_STATUS_H > kFont8x16GlyphH)
	                          ? ((ST7789_STATUS_H - kFont8x16GlyphH) / 2u) : 0u;

	draw_string_small(tx, ty, text, COLOR_WHITE, bg);
}

/* --------------------------------------------------------------------------
 * Render the large numeric value + units
 * --------------------------------------------------------------------------*/
void ST7789Display::render_numeric(float value, uint8_t decimal_places,
                                   const char *units)
{
	/* Format the number into a buffer */
	char numbuf[24];
	const int dp = (decimal_places > 4) ? 4 : static_cast<int>(decimal_places);

	if (fabsf(value) >= 10000.0f) {
		/* Scientific notation for very large numbers */
		snprintf(numbuf, sizeof(numbuf), "%.2e", static_cast<double>(value));
	} else {
		snprintf(numbuf, sizeof(numbuf), "%.*f", dp, static_cast<double>(value));
	}

	/* Clear the numeric area */
	const uint16_t num_area_y = ST7789_STATUS_H + 4u;
	const uint16_t num_area_h = static_cast<uint16_t>(_h - num_area_y);
	fill_rect(0, num_area_y, _w, num_area_h, COLOR_BLACK);

	/* Centre the large number horizontally */
	const uint16_t nw = string_pixel_width_large(numbuf);
	const uint16_t nx = (_w > nw) ? ((_w - nw) / 2u) : 0u;

	/* Vertically centre in the lower two-thirds of the screen */
	const uint16_t total_h = static_cast<uint16_t>(kFontLargeGlyphH +
	                          (strnlen(units, 7) > 0u ? (kFont8x16GlyphH + 6u) : 0u));
	const uint16_t avail_h = static_cast<uint16_t>(num_area_h - 20u);
	const uint16_t ny      = num_area_y + ((avail_h > total_h)
	                              ? ((avail_h - total_h) / 2u) : 0u);

	draw_string_large(nx, ny, numbuf, COLOR_WHITE, COLOR_BLACK);

	/* Draw units below the number in smaller font */
	if (units && units[0] != '\0') {
		const uint16_t uy  = static_cast<uint16_t>(ny + kFontLargeGlyphH + 6u);
		const size_t   ul  = strnlen(units, 7);
		const uint16_t uw  = static_cast<uint16_t>(ul * kFont8x16GlyphW);
		const uint16_t ux  = (_w > uw) ? ((_w - uw) / 2u) : 0u;
		draw_string_small(ux, uy, units, COLOR_LGRAY, COLOR_BLACK);
	}
}

/* --------------------------------------------------------------------------
 * Full-screen render from a display_command message
 * --------------------------------------------------------------------------*/
void ST7789Display::render_full(const display_command_s &cmd)
{
	if (cmd.clear_display) {
		fill_rect(0, 0, _w, _h, COLOR_BLACK);
	}

	render_status_bar(cmd.status_text, cmd.status_color);

	char units[9];
	memcpy(units, cmd.units, 8);
	units[8] = '\0';

	render_numeric(cmd.numeric_value, cmd.decimal_places, units);
}

/* --------------------------------------------------------------------------
 * Work loop
 * --------------------------------------------------------------------------*/
void ST7789Display::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	if (!_initialized) { return; }

	display_command_s cmd;

	if (_cmd_sub.update(&cmd)) {
		render_full(cmd);
	}
}

/* --------------------------------------------------------------------------
 * Status
 * --------------------------------------------------------------------------*/
int ST7789Display::print_status()
{
	PX4_INFO("ST7789 Display Driver");
	PX4_INFO("  SPI bus  : %ld @ %ld kHz", _spi_bus, _spi_freq / 1000u);
	PX4_INFO("  GPIO RES : 0x%08X", (unsigned int)_gpio_res);
	PX4_INFO("  GPIO DC  : 0x%08X", (unsigned int)_gpio_dc);
	PX4_INFO("  GPIO CS  : 0x%08X", (unsigned int)_gpio_cs);
	PX4_INFO("  GPIO BLK : 0x%08X", (unsigned int)_gpio_blk);
	PX4_INFO("  Rotation : %ld", _param_rotate.get());
	PX4_INFO("  Init ok  : %s", _initialized ? "yes" : "no");
	return 0;
}

/* --------------------------------------------------------------------------
 * Module entry point
 * --------------------------------------------------------------------------*/
extern "C" __EXPORT int st7789_display_main(int argc, char *argv[])
{
	return ST7789Display::main(argc, argv);
}
