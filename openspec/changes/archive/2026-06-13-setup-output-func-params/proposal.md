## Why

`rc.board_defaults` unconditionally assigns all 8 PWM output channels to Motor1–8 on every boot, so outputs are "hot" before any SETUP is run. `apply_group_mode()` sets the timer protocol and channel limits when the user commits a group configuration, but never writes `PWM_MAIN_FUNC{N}`, so the output driver's function map never changes — disabling a group in SETUP has no effect on which channels the driver routes.

## What Changes

- **Startup**: Remove the explicit `param set PWM_MAIN_FUNC1-8 101-108` assignments from `rc.board_defaults`. Channels start unconfigured (function = 0, no output) until SETUP is committed.
- **`apply_group_mode()`**: Write `PWM_MAIN_FUNC{first..last}` for every group commit:
  - Mode ≠ 0 (DShot or PWM): set function to `Motor{ch}` (value = 100 + channel number)
  - Mode = 0 (disabled): set function to 0 (disabled)
  - Restructure the disabled early-return so the FUNC write and SVT param update always execute; only the MIN/MAX/DIS/FAIL block is skipped for disabled mode.

## Capabilities

### New Capabilities

- `output-func-management`: `PWM_MAIN_FUNC` params are managed by SETUP; disabled groups explicitly zero their channel functions

### Modified Capabilities

(none — protocol/limits behaviour of `apply_group_mode` is unchanged)

## Impact

- `boards/crystal/nuc-h7xx/init/rc.board_defaults`: remove or zero `PWM_MAIN_FUNC1-8` assignments
- `src/modules/servo_test/ServoTest.cpp` `apply_group_mode()`: add FUNC write loop; restructure disabled-mode early-return

## Non-goals

- Managing `UAVCAN_EC_FUNC1-8` (UAVCAN output routing is independent; SVT_UAVCAN_EN controls it)
- Per-channel function override (all channels in a group always map to sequential Motor functions)
- Triggering an output driver restart after param changes (the driver picks up param updates normally; a reboot may be required for protocol changes, as before)
