## Context

Three bugs in `src/modules/servo_test/ServoTest.cpp`:

1. **CAN param type mismatch** (`exit_setup()`): `param_set(p, &en)` where `en` is `bool` (1 byte) but `SVT_UAVCAN_EN` is PARAM_TYPE_INT32 (4 bytes). The function reads 3 bytes past `en` from the stack, producing a garbage int32 value that is almost always non-zero → CAN appears permanently enabled.

2. **Graceful stop resets throttle** (STOPPING ramp completion in `Run()`): `throttle-display-fix` added `_target_throttle = 0.0f` to both stop paths. The immediate path is correct (safety reset). The graceful path should preserve setpoint so the operator can restart at the same throttle.

3. **Encoder during button hold** (RUNNING encoder block): `enc.delta` is applied to `_target_throttle` unconditionally; the `push_rising` handler (which initiates stop) is processed after the delta — in the same loop iteration the encoder may also have moved.

## Goals / Non-Goals

**Goals:**
- CAN enabled/disabled state persists correctly through SETUP commit
- After graceful stop: CONFIG shows the last running throttle, ready to restart
- After e-stop: CONFIG shows 0% (safety reset unchanged)
- Encoder is frozen while the stop button is physically depressed

**Non-Goals:**
- Visual feedback during stop button hold
- Changing when stop fires (still `push_rising`)

## Decisions

### CAN fix: int32_t local, not bool
```cpp
// Before (BUG):
bool en = _pending_uavcan;
param_set(p, &en);           // reads 4 bytes, only 1 is valid

// After (FIX):
int32_t en = _pending_uavcan ? 1 : 0;
param_set(p, &en);           // reads exactly 4 bytes, correct value
```
No API change; `_param_uavcan_en` (`ParamBool`) still reads correctly because `ParamBool` treats any non-zero int32 as true.

### Graceful stop: remove _target_throttle reset from ramp completion
The `_target_throttle = 0.0f` on line 260 of `Run()` (STOPPING ramp completion) is removed. The display jump that motivated it in `throttle-display-fix` is already addressed by the STARTING display fix (showing `_target_throttle` during ramp-up). The STOPPING → CONFIG transition now shows the last-set throttle, which is the intended behaviour.

The immediate stop path in `stop_running()` keeps its reset — an e-stop is an abort, not a pause.

### Encoder gate: _run_push_held flag in RUNNING
```cpp
// RUNNING encoder block:
if (enc.delta != 0 && !_run_push_held) {
    _target_throttle = clampf(...);
}
if (enc.push_rising) {
    _run_push_held = true;
    stop_running(false);
}
if (enc.push_falling) {
    _run_push_held = false;
}
if (enc.k0_rising) {
    stop_running(true);
}
```
The flag is also cleared in `stop_running(immediate=true)` (state leaves RUNNING immediately, so no `push_falling` will arrive in RUNNING context). No need to clear in `start_running()` — the flag starts false and is scoped to RUNNING.

## Risks / Trade-offs

- [CAN fix] → Writing int32 value 0 or 1 is unambiguous. Existing boards with a garbage param value will correct on next SETUP commit or param reset.
- [Preserved throttle on graceful stop] → User restarting at a high throttle may be surprised. Mitigated by the existing ramp-up filter which provides smooth acceleration from 0.
- [Encoder gate] → Encoder input lost during button hold (typically <200 ms). Acceptable; the operator is intentionally stopping.
