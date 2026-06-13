## 1. Board Defaults — Zero FUNC Params on Boot

- [x] 1.1 In `boards/crystal/nuc-h7xx/init/rc.board_defaults`: replace the eight `param set PWM_MAIN_FUNC{1..8} {101..108}` lines with eight `param set PWM_MAIN_FUNC{1..8} 0` lines so every boot starts with all channels unconfigured (guarantees a clean state after first flash)

## 2. apply_group_mode() — Write FUNC Params

- [x] 2.1 In `src/modules/servo_test/ServoTest.cpp` `apply_group_mode()`: after the `group_channels()` call, add a FUNC write loop that iterates `first..last` and sets `PWM_MAIN_FUNC{ch}` to `(mode != 0) ? (100 + ch) : 0`
- [x] 2.2 In the same function: restructure the disabled early-return so that the FUNC write loop (2.1) and the final `SVT_GRP_X_MODE` + `updateParams()` block always execute; guard only the MIN/MAX/DIS/FAIL write block with `if (mode_is_dshot(mode) || mode_is_pwm(mode))`

## 3. Build Verification

- [x] 3.1 Run `make crystal_nuc-h7xx_default` and confirm clean build with no errors or warnings in the changed files
