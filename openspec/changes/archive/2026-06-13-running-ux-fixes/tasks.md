## 1. CAN Param Type Fix

- [x] 1.1 In `src/modules/servo_test/ServoTest.cpp` `exit_setup()`: change `bool en = _pending_uavcan;` to `int32_t en = _pending_uavcan ? 1 : 0;` so `param_set()` receives a correctly-typed 4-byte value

## 2. Graceful Stop Preserves Setpoint

- [x] 2.1 In `ServoTest::Run()` STOPPING ramp completion block (`if (_output_throttle < 0.01f)`): remove the `_target_throttle = 0.0f;` line — only `_output_throttle` is zeroed here; `stop_running(immediate=true)` already resets `_target_throttle` for e-stop

## 3. Encoder Gate During Stop Button Hold

- [x] 3.1 In `src/modules/servo_test/ServoTest.hpp`: add `bool _run_push_held{false};` to the RUNNING state member section (alongside other run-state flags)
- [x] 3.2 In `ServoTest::Run()` RUNNING encoder block: gate the `enc.delta` handler — change `if (enc.delta != 0)` to `if (enc.delta != 0 && !_run_push_held)`
- [x] 3.3 In the same RUNNING encoder block: in the `push_rising` handler, add `_run_push_held = true;` before `stop_running(false)`
- [x] 3.4 In the same RUNNING encoder block: add a `push_falling` handler — `if (enc.push_falling) { _run_push_held = false; }`
- [x] 3.5 In `ServoTest::stop_running()` immediate branch: add `_run_push_held = false;` to ensure the flag is cleared when e-stop fires (state leaves RUNNING without waiting for `push_falling`)

## 4. Build Verification

- [x] 4.1 Run `make crystal_nuc-h7xx_default` and confirm clean build with no errors or warnings in the changed files
