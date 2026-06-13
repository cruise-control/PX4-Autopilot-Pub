## Why

Three independent bugs in the `servo_test` module degrade usability: CAN can never be disabled from the UI due to a type mismatch in `param_set`, graceful stop incorrectly resets the throttle setpoint to zero (making restart require re-dialling), and turning the encoder while pressing the stop button alters the setpoint in an uncontrolled way.

## What Changes

- **CAN disable bug**: In `exit_setup()`, change `bool en` → `int32_t en` before calling `param_set()`. A `bool` is 1 byte; `param_set` reads 4 bytes (int32), picking up 3 garbage stack bytes that make the stored value non-zero even when the user chose "disabled".
- **Graceful stop preserves setpoint**: Remove `_target_throttle = 0.0f` from the STOPPING ramp completion block in `Run()`. E-stop (`stop_running(immediate=true)`) still resets to zero. Graceful stop now returns to CONFIG showing the last-used throttle, ready to restart.
- **Encoder lock during stop button hold**: Add a `_run_push_held` flag in RUNNING state. Set it on `push_rising` (alongside initiating the graceful stop); clear it on `push_falling`. Gate `enc.delta` processing so throttle updates are suppressed while the button is depressed.

## Capabilities

### New Capabilities

- `encoder-gate-on-stop`: Encoder delta is ignored in RUNNING state while the main button is held

### Modified Capabilities

- `throttle-display`: Graceful stop no longer resets `_target_throttle` to zero (only e-stop does)

## Impact

- `src/modules/servo_test/ServoTest.hpp`: add `bool _run_push_held{false}`
- `src/modules/servo_test/ServoTest.cpp`:
  - `exit_setup()`: `bool en` → `int32_t en`
  - `Run()` STOPPING ramp completion: remove `_target_throttle = 0.0f`
  - `Run()` RUNNING encoder block: add `_run_push_held` guard on `enc.delta` and set/clear on push events
  - `stop_running(immediate=true)` path: no change needed (already resets `_target_throttle`)

## Non-goals

- Encoder lock during K0 hold in CONFIG (K0 already cancels `_push_pending`)
- Visual indicator that the stop button is held
- Changing the e-stop (K0) behaviour
