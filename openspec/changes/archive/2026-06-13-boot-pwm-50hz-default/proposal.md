## Why

AM32 ESCs require a valid PWM signal at startup to identify the protocol and complete their arming sequence. If the board boots into DShot mode (persisted from a prior SETUP commit), the ESCs may not initialize reliably. Defaulting every boot to PWM 50Hz ensures the ESC startup procedure runs correctly on every power cycle. The operator can then switch to their desired protocol via SETUP.

Additionally, the current `param set-default` statements for TIM, channel limits (MIN/MAX/DIS/FAIL), and FUNC are overridden by flash values written by `apply_group_mode()` — so after a DShot SETUP, subsequent boots continue in DShot mode (or, after `setup-output-func-params`, with DShot channel limits but no FUNC mapping). Only `param set` (explicit override) ensures a clean PWM state on every boot.

## What Changes

All changes are in `rc.board_defaults` and affect `param set` / `param set-default` variants:

- **`PWM_MAIN_TIM0/1/2`**: Change from `param set-default 50` to `param set 50` — forces PWM 50Hz on every boot regardless of flash
- **`PWM_MAIN_FUNC1-8`**: Change from `param set 0` to `param set 101-108` — maps all channels to Motor1-8 on every boot so ESCs receive signals
- **`PWM_MAIN_MIN/MAX/DIS/FAIL{1-8}`**: Change from `param set-default` to `param set` with the same PWM values — ensures valid pulse widths even after a prior DShot SETUP wrote DShot limits (min=0, dis=0) to flash
- **`SVT_GRP_A/B/C_MODE`**: Change from `param set-default` to `param set 4` (PWM 50Hz) — UI reflects the true hardware state on boot; all three groups active in PWM 50Hz

## Capabilities

### New Capabilities

- `boot-pwm-state`: Board boots into a fully defined PWM 50Hz state on every power cycle, enabling ESC startup procedures

### Modified Capabilities

(none — SETUP protocol commit behaviour is unchanged; only the boot baseline changes)

## Impact

- `boards/crystal/nuc-h7xx/init/rc.board_defaults`: 35 param statement changes (set-default → set; values where noted)

## Non-goals

- Changing what SETUP commits (apply_group_mode behaviour unchanged)
- Auto-running SETUP on first boot
- Persisting the protocol choice across reboots (operator must re-run SETUP each session — this is intentional)
