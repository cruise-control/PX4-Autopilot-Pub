## 1. Add _push_pending Member

- [x] 1.1 In `src/modules/servo_test/ServoTest.hpp`: add `bool _push_pending{false};` to the "K0 hold detection" member block (alongside `_k0_held`)

## 2. Update CONFIG Encoder Handling

- [x] 2.1 In `src/modules/servo_test/ServoTest.cpp` `Run()` CONFIG case: change the `push_rising` handler from calling `start_running()` to setting `_push_pending = true`
- [x] 2.2 In the same CONFIG case: add a `push_falling` handler — `if (enc.push_falling && _push_pending) { _push_pending = false; start_running(); }`
- [x] 2.3 In the same CONFIG case: in the `k0_rising` handler, add `_push_pending = false;` to cancel a pending start when the operator switches to K0 hold

## 3. Clear _push_pending on State Exit

- [x] 3.1 In `ServoTest::enter_setup()`: add `_push_pending = false;` so a pending start cannot survive a CONFIG→SETUP→CONFIG round-trip

## 4. Build Verification

- [x] 4.1 Run `make crystal_nuc-h7xx_default` and confirm clean build with no errors or warnings in the changed files
