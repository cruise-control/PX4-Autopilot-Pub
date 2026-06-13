## 1. EC11 Driver Fix

- [x] 1.1 In `src/drivers/ec11_rotary_encoder/EC11RotaryEncoder.cpp` `process_encoder_step()`: change `_position += step*4` and `_delta += step*4` to `_position += step` and `_delta += step`

## 2. servo_test Parameter

- [x] 2.1 Add `SVT_THR_STEP` (float, min=0.0001, max=0.02, default=0.0025, unit=none) to `src/modules/servo_test/module.yaml`
- [x] 2.2 Add `(ParamFloat<px4::params::SVT_THR_STEP>) _param_thr_step` to `DEFINE_PARAMETERS` block in `src/modules/servo_test/ServoTest.hpp`
- [x] 2.3 Add `int32_t _menu_delta_acc{0}` member to `ServoTest.hpp` for menu navigation accumulator

## 3. servo_test Throttle Path

- [x] 3.1 In `ServoTest::Run()` CONFIG encoder handling: replace `enc.delta * 0.02f` with `static_cast<float>(enc.delta) * _param_thr_step.get()`
- [x] 3.2 In `ServoTest::Run()` RUNNING encoder handling: same replacement for the throttle delta

## 4. servo_test Menu Navigation

- [x] 4.1 In `ServoTest::Run()` SETUP MENU encoder handling: replace direct `_menu_item = (_menu_item + enc.delta + 4) % 4` with accumulator logic — add `enc.delta` to `_menu_delta_acc`, then advance `_menu_item` by ±1 for each ±4 consumed
- [x] 4.2 In SETUP GROUP_A/B/C sub-page encoder handling: replace direct `_sub_item = (_sub_item + enc.delta + 8) % 8` with same accumulator pattern (reuse `_menu_delta_acc`)
- [x] 4.3 In SETUP UAVCAN_PAGE encoder handling: apply accumulator — toggle `_pending_uavcan` only when `|_menu_delta_acc| >= 4`
- [x] 4.4 Reset `_menu_delta_acc = 0` in `enter_setup()` and in the `exit_setup()` / state-transition paths so partial events don't carry over between states

## 5. Board Defaults

- [x] 5.1 Add `param set-default SVT_THR_STEP 0.0025` to `boards/crystal/nuc-h7xx/init/rc.board_defaults` (before `servo_test start`)

## 6. Build Verification

- [x] 6.1 Run `make crystal_nuc-h7xx_default` and confirm clean build with no errors on new param
