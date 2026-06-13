## 1. SVT_THR_STEP Default

- [x] 1.1 In `src/modules/servo_test/module.yaml`: change `SVT_THR_STEP` `default` from `0.0025` to `0.000625` and update the long description to read "Default 0.000625 gives 0.25% (0.0025) per detent click."
- [x] 1.2 In `boards/crystal/nuc-h7xx/init/rc.board_defaults`: change `param set-default SVT_THR_STEP 0.0025` to `param set-default SVT_THR_STEP 0.000625`

## 2. Display Decimal Places

- [x] 2.1 In `src/modules/servo_test/ServoTest.cpp` `publish_display()` CONFIG case: change `d.decimal_places = 0` (throttle readout) to `d.decimal_places = 2`
- [x] 2.2 In `publish_display()` RUNNING case: change `d.decimal_places = 0` (throttle readout) to `d.decimal_places = 2`

## 3. Fix STARTING Ramp Display Jump

- [x] 3.1 In `publish_display()` RUNNING case: replace `d.numeric_value = _output_throttle * 100.0f` with `d.numeric_value = ((_ramp_state == RampState::STARTING) ? _target_throttle : _output_throttle) * 100.0f` so the display shows the commanded value during ramp-up

## 4. Reset Throttle on Stop

- [x] 4.1 In `ServoTest::stop_running()` immediate branch (`if (immediate || !_param_ramp_en.get())`): add `_target_throttle = 0.0f;` after `_output_throttle = 0.0f;`
- [x] 4.2 In `ServoTest::Run()` STOPPING ramp completion block (`if (_output_throttle < 0.01f)`): add `_target_throttle = 0.0f;` alongside the existing `_output_throttle = 0.0f;` assignment

## 5. Build Verification

- [x] 5.1 Run `make crystal_nuc-h7xx_default` and confirm clean build with no errors or warnings related to changed files
