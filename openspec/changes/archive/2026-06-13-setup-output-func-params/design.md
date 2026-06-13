## Context

`PWM_MAIN_FUNC{N}` tells the output driver which logical function (Motor1=101, Motor2=102, …) maps to physical output channel N. Without this mapping, `actuator_test` messages targeting `FUNCTION_MOTOR{N}` are not routed to any hardware pin.

Currently:
- `rc.board_defaults` hard-sets all 8 FUNC params to 101–108 on every boot via `param set` (not `param set-default`), overwriting any previously committed SETUP config.
- `apply_group_mode()` writes TIM + MIN/MAX/DIS/FAIL but not FUNC, so SETUP commits have no effect on the driver's function routing.
- The disabled-mode early-return exits before writing SVT_GRP_X_MODE too (it does write it inline, then returns), and it skips any FUNC update entirely.

## Goals / Non-Goals

**Goals:**
- Boot with all FUNC = 0; channels are inert until SETUP runs
- SETUP commit writes FUNC for the group's channels (active = Motor function, disabled = 0)
- Disabled mode properly zeroes both TIM and FUNC

**Non-Goals:**
- UAVCAN_EC_FUNC management (orthogonal, stays as-is)
- Auto-applying SETUP on first boot (user must run SETUP once)

## Decisions

### Remove param set PWM_MAIN_FUNC1-8 lines from rc.board_defaults
These are `param set` (not `param set-default`) so they actively overwrite flash on every boot. Removing them allows the flash-retained values from a prior SETUP to survive a reboot. On a truly fresh device, the factory default for FUNC params is 0, which is the desired state.

Note: the `UAVCAN_EC_FUNC1-8` lines remain since they're unrelated to this change.

### Restructure apply_group_mode() — remove early return for disabled mode
The current disabled branch writes `SVT_GRP_X_MODE` inline and returns. Restructuring:
1. Always write `PWM_MAIN_TIM{group}` (including 0 for disabled)
2. Always write `PWM_MAIN_FUNC{first..last}` (`100 + ch` or `0`)
3. Write MIN/MAX/DIS/FAIL only when `mode_is_dshot` or `mode_is_pwm`
4. Always write `SVT_GRP_X_MODE` at the end
5. Always call `updateParams()`

This eliminates the early return and makes all three paths (disabled, dshot, pwm) share the same TIM + FUNC + SVT writes.

### Motor function value: 100 + channel (1-based)
PX4 defines `FUNCTION_MOTOR1 = 101`, `FUNCTION_MOTOR2 = 102`, … `FUNCTION_MOTOR8 = 108` as sequential enum values. Channel `ch` (1-based) maps to function `100 + ch`. This is consistent with how `publish_throttle()` already addresses motors:
```cpp
t.function = static_cast<uint16_t>(actuator_test_s::FUNCTION_MOTOR1 + (ch - 1));
// FUNCTION_MOTOR1 = 101, so this is 101 + (ch - 1) = 100 + ch ✓
```

## Risks / Trade-offs

- [First-boot with no SETUP] → outputs are all disabled until SETUP is committed. The operator must run SETUP once. Acceptable for a bench tester — safer than having all outputs hot on boot.
- [Existing boards] → after flashing, the first boot no longer forces FUNC = 101-108. If flash has garbage values (e.g., from a previous different firmware), outputs may be misrouted until SETUP is run. Mitigation: rc.board_defaults could add `param set PWM_MAIN_FUNC1-8 0` explicitly to guarantee a clean slate on first flash. This is addressed in the tasks.
- [Output driver param update] → changing FUNC params at runtime via `apply_group_mode` relies on the PWM/DShot output driver subscribing to parameter updates. This is the standard PX4 mechanism; a reboot guarantees the new FUNC values take effect for the timer protocol change anyway.
