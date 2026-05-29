# PX4 Driver: 2.4" TFT + EC11 Combo Board
## Amazon B0GW29KWS9 — ST7789 SPI Display + EC11 Rotary Encoder

This package provides two PX4 loadable driver modules for the combined
2.4-inch TFT LCD + EC11 rotary encoder board.

---

## Hardware Overview

| Pin   | Function                          | Direction |
|-------|-----------------------------------|-----------|
| GND   | Ground                            | —         |
| VCC   | 3.3 V supply                      | In        |
| SCL   | SPI SCLK (display)                | Out       |
| SDA   | SPI MOSI (display)                | Out       |
| RES   | Display hardware reset            | Out       |
| DC    | Display data/command select       | Out       |
| CS    | Display chip-select (active low)  | Out       |
| BLK   | Display backlight PWM / on-off    | Out       |
| A     | Encoder phase A (quadrature)      | In        |
| B     | Encoder phase B (quadrature)      | In        |
| PUSH  | Encoder shaft push-button         | In        |
| K0    | Independent back/menu button      | In        |

All inputs (A, B, PUSH, K0) are active-LOW; enable internal pull-ups.

---

## Module 1 — `ec11_rotary_encoder`

Publishes **`rotary_encoder_event`** uORB topic on every encoder step or
button state change.

### Fields published
| Field          | Type    | Description                             |
|----------------|---------|-----------------------------------------|
| timestamp      | uint64  | HRT timestamp [µs]                      |
| position       | int32   | Absolute cumulative step count          |
| delta          | int8    | Steps since last publish (+CW / −CCW)  |
| button_push    | bool    | Encoder shaft button current state      |
| button_k0      | bool    | K0 back/menu button current state       |
| push_rising    | bool    | True on press edge (active-low → low)   |
| push_falling   | bool    | True on release edge                    |
| k0_rising      | bool    | True on K0 press edge                   |
| k0_falling     | bool    | True on K0 release edge                 |

### Parameters
| Parameter          | Default | Description                        |
|--------------------|---------|------------------------------------|
| EC11_GPIO_PIN_A    | 0       | GPIO word for encoder phase A      |
| EC11_GPIO_PIN_B    | 0       | GPIO word for encoder phase B      |
| EC11_GPIO_PUSH     | 0       | GPIO word for PUSH button          |
| EC11_GPIO_K0       | 0       | GPIO word for K0 button            |
| EC11_DEBOUNCE_US   | 5000    | Button debounce period [µs]        |

### Usage
```
ec11_rotary_encoder start
ec11_rotary_encoder status
ec11_rotary_encoder stop
```

---

## Module 2 — `st7789_display`

Subscribes to **`display_command`** uORB topic and renders a two-zone
layout on the 240×320 ST7789 display:

```
┌──────────────────────────────────────┐
│          STATUS MESSAGE               │  ← 36 px status bar
│          (colour-coded bg)            │    white/green/yellow/red
├──────────────────────────────────────┤
│                                      │
│                                      │
│         1 2 3 4 . 5 6                │  ← Large 16×32 font
│                                      │
│              m/s                     │  ← Units (small font)
│                                      │
└──────────────────────────────────────┘
```

### Fields subscribed (`display_command`)
| Field          | Type     | Description                            |
|----------------|----------|----------------------------------------|
| numeric_value  | float32  | Value to render in large font          |
| decimal_places | uint8    | Decimal places (0–4)                   |
| units          | char[8]  | Unit string e.g. "m/s"                 |
| status_text    | char[32] | Status bar message                     |
| status_color   | uint8    | 0=blue 1=green 2=yellow 3=red          |
| backlight_on   | bool     | Backlight control                      |
| clear_display  | bool     | Force full clear before redraw         |

### Parameters
| Parameter           | Default | Description                         |
|---------------------|---------|-------------------------------------|
| ST7789_SPI_BUS      | 1       | SPI bus number                      |
| ST7789_SPI_FREQ     | 40000   | SPI clock [kHz]                     |
| ST7789_GPIO_RES     | 0       | GPIO word for RES pin               |
| ST7789_GPIO_DC      | 0       | GPIO word for DC pin                |
| ST7789_GPIO_CS      | 0       | GPIO word for CS pin                |
| ST7789_GPIO_BLK     | 0       | GPIO word for BLK pin (0=always on) |
| ST7789_ROTATE       | 0       | 0=Portrait 1=Landscape              |

### Usage
```
st7789_display start
st7789_display status
st7789_display stop
```

---

## Integration into a PX4 Board Config

1. Copy `src/drivers/ec11_rotary_encoder/` and
   `src/drivers/st7789_display/` into your PX4 source tree under
   `src/drivers/`.

2. Copy `msg/rotary_encoder_event.msg` and `msg/display_command.msg`
   into `msg/`.

3. Add both messages to `msg/CMakeLists.txt`:
   ```cmake
   set(msg_files
       ...
       rotary_encoder_event.msg
       display_command.msg
   )
   ```

4. Add both drivers to your board's `default.cmake`:
   ```cmake
   DRIVERS
       ...
       ec11_rotary_encoder
       st7789_display
   ```

5. Set parameters in your board's `param_defaults.yaml` or via the
   `param set` shell command with the correct GPIO configuration words
   for your board's pin mapping.

6. Add to your startup script (`rc.board_defaults` or similar):
   ```sh
   ec11_rotary_encoder start
   st7789_display start
   ```

---

## GPIO Configuration Words (STM32)

GPIO words are constructed from NuttX `stm32_gpio.h` macros:

```c
// Example: PA0 as input with pull-up and EXTI interrupt
#define MY_PIN (GPIO_INPUT | GPIO_PULLUP | GPIO_EXTI | GPIO_PORTA | GPIO_PIN0)
```

Set `EC11_GPIO_PIN_A` etc. to the integer value of the appropriate macro
for your specific board pin mapping.

---

## File Structure

```
src/drivers/
├── ec11_rotary_encoder/
│   ├── EC11RotaryEncoder.hpp
│   ├── EC11RotaryEncoder.cpp
│   ├── CMakeLists.txt
│   └── module.yaml
└── st7789_display/
    ├── ST7789Display.hpp
    ├── ST7789Display.cpp
    ├── font_data.hpp          ← Embedded 8×16 and 16×32 bitmap fonts
    ├── CMakeLists.txt
    └── module.yaml

msg/
├── rotary_encoder_event.msg
└── display_command.msg
```
