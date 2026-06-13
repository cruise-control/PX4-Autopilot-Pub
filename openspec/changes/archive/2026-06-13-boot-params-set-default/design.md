## Context

`boot-pwm-50hz-default` introduced `param set` (unconditional write) for all startup params so the board always boots into PWM 50Hz regardless of flash. This was motivated by AM32 ESC arming: the ESC needs a valid PWM signal on power-up to identify the protocol. The side effect is that any param the operator deliberately saves (via GCS, MAVLink, or `param save` on the NSH shell) is silently overwritten on every reboot.

`param set-default` writes the in-memory value only when no flash value exists for that parameter. On a factory-fresh board, behavior is identical to `param set`. On a board with saved params, the flash values win.

## Goals / Non-Goals

**Goals:**
- Operator-saved params persist across reboots
- Factory-fresh board still boots into PWM 50Hz with Motor1-8 mapped (defaults apply)
- No change to `apply_group_mode()` behavior

**Non-Goals:**
- Changing the default values (same 1100/1900/1000/1000 µs, PWM 50Hz, Motor1-8)
- Adding a separate "reset to factory" command
- Guaranteeing AM32 PWM arming on every reboot (that guarantee is relinquished by this change)

## Decisions

### Use `param set-default` for all affected startup params

`param set` → `param set-default` for:
- `PWM_MAIN_TIM0/1/2`
- `PWM_MAIN_FUNC1-8`
- `PWM_MAIN_MIN/MAX/DIS/FAIL{1-8}`
- `SVT_GRP_A/B/C_MODE`

All other params in `rc.board_defaults` already use `param set-default` and are unaffected.

### Accept that reboot after a DShot SETUP may not auto-PWM-arm the ESC

With `param set`, every reboot reset TIM/FUNC/limits to PWM values, ensuring AM32 could re-arm. With `param set-default`, if the operator committed DShot via SETUP and the board reboots, it stays in DShot. The operator must either power-cycle the ESC independently or re-run SETUP explicitly. This is acceptable given the bench-use context.

## Risks / Trade-offs

- [AM32 may not arm after reboot into DShot] → Operator power-cycles ESC or re-runs SETUP. Not a regression vs. DShot-capable GCS-configured boards.
- [Stale flash from prior DShot session causes unexpected state] → Operator can always clear params with `param reset` on the NSH shell or via GCS, then reboot to land on defaults.
