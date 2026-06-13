## Why

`boot-pwm-50hz-default` used `param set` (unconditional override) for all startup params, which meant any change the operator made via QGC, MAVLink param editor, or the NSH shell was silently wiped on the next reboot. Switching to `param set-default` preserves flash-stored values so intentional local changes survive across power cycles.

## What Changes

- All `param set` lines in `rc.board_defaults` for PWM/SVT startup params are changed to `param set-default`
- On a board with no flash values (factory state), behaviour is identical to before
- On a board where the operator has explicitly saved a param (e.g. via GCS or `param save`), that value is retained across reboots instead of being overwritten

## Capabilities

### New Capabilities

(none)

### Modified Capabilities

- `boot-pwm-state`: The boot-time param override strategy changes from unconditional (`param set`) to default-only (`param set-default`). The shipped defaults remain the same values (PWM 50Hz, Motor1-8, 1100/1900/1000/1000 limits); they are no longer forced on every boot.

## Impact

- `boards/crystal/nuc-h7xx/init/rc.board_defaults`: Change `param set` to `param set-default` for `PWM_MAIN_TIM0/1/2`, `PWM_MAIN_FUNC1-8`, `PWM_MAIN_MIN/MAX/DIS/FAIL{1-8}`, `SVT_GRP_A/B/C_MODE`

## Non-goals

- Changing the default values themselves (1100/1900/1000/1000 µs, PWM 50Hz, Motor1-8 — all unchanged)
- Changing `apply_group_mode()` behaviour (SETUP commits still write to flash as before)
- Adding any new mechanism to persist or restore params
