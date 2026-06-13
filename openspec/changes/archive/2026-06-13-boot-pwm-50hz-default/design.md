## Context

`rc.board_defaults` runs every boot. It currently uses `param set-default` for TIM and channel limits, which only applies when no value is stored in flash. After `apply_group_mode()` writes DShot params to flash (via `param_set()`), subsequent boots keep DShot values.

Two categories of params to change:

| Param group | Current | Problem | Fix |
|---|---|---|---|
| `PWM_MAIN_TIM0/1/2` | `param set-default 50` | Flash DShot value (-4) overrides default | `param set 50` |
| `PWM_MAIN_FUNC1-8` | `param set 0` (from setup-output-func-params) | Channels unmapped; ESC gets no signal | `param set 101-108` |
| `PWM_MAIN_MIN/MAX/DIS/FAIL{1-8}` | `param set-default` | DShot DIS=0 persists in flash; ESC gets no pulse | `param set` (same values) |
| `SVT_GRP_A/B/C_MODE` | `param set-default 2/0/0` | UI shows stale DShot config; inconsistent with TIM=50 | `param set 4/4/4` |

## Goals / Non-Goals

**Goals:**
- Every boot: outputs are in PWM 50Hz with all channels mapped and valid pulse widths
- UI (servo_test CONFIG display) matches the actual hardware state on boot
- AM32 arming sequence can complete on every power cycle

**Non-Goals:**
- Persisting protocol choice across reboots (by design: each session starts fresh)
- Changing when SETUP takes effect (still immediate, within the same session)
- Managing UAVCAN output (SVT_UAVCAN_EN stays as `param set-default`)

## Decisions

### param set (not param set-default) for all affected params
`param set-default` is a "fill in if empty" instruction. `param_set()` in `apply_group_mode()` explicitly writes to the param store (flash-backed), so flash always has a non-default value after the first SETUP commit. `param set` in the rcS script unconditionally overrides the in-memory value after load from flash, achieving the desired reset.

### All three timer groups set to PWM 50Hz (not just Group A)
The user said "all direct outputs." Groups B and C (TIM4/TIM2) are also direct outputs. Defaulting all three groups to PWM 50Hz keeps the board state symmetric and consistent regardless of which groups the operator plans to use.

### SVT_GRP mode = 4 (not 0)
Mode 4 is "PWM 50Hz" which matches the actual TIM value. Setting it to 0 (disabled) would show "disabled" in the UI even though the output is actively driven in PWM mode. Mode 4 is accurate and gives the operator a sensible starting point to SETUP from.

### Flash wear
`param set` in the rcS script modifies the in-memory param value. The PX4 parameter save mechanism writes to flash when values change. If a prior SETUP committed DShot values, the next boot will detect the mismatch and write the PWM values — one flash write cycle per session that ends with a SETUP commit. For a bench tester this is acceptable. PX4's parameter storage uses wear-leveling, further mitigating concern.

## Risks / Trade-offs

- [Protocol lost on reboot] → The operator must re-run SETUP after every power cycle to use DShot. This is intentional — it ensures the AM32 startup procedure always runs, and the operator is always explicit about their test configuration.
- [35 param set lines in rcS] → Verbose but explicit. Splitting into groups with comments maintains readability.
