## 1. Board Configuration

- [x] 1.1 Add `CONFIG_DRIVERS_POWER_MONITOR_INA238=y` to `boards/crystal/nuc-h7xx/default.px4board`
- [x] 1.2 Add `ina238 start -X -b <bus> -a <addr>` to `boards/crystal/nuc-h7xx/init/rc.board_defaults` (confirm I2C bus and address from hardware schematic)
- [x] 1.3 Remove `param set-default SVT_RPM_STEP`, `SVT_RPM_MAX`, `SVT_NUM_MOT` from `rc.board_defaults`
- [x] 1.4 Add new param defaults to `rc.board_defaults`: `SVT_RAMP_EN 1`, `SVT_RAMP_TAU 0.5`, `SVT_GRP_A_MODE 2` (DShot300), `SVT_GRP_B_MODE 0` (disabled), `SVT_GRP_C_MODE 0` (disabled), `SVT_UAVCAN_EN 0`

## 2. Parameter Definitions

- [x] 2.1 Remove `SVT_RPM_STEP`, `SVT_RPM_MAX`, `SVT_NUM_MOT` from `src/modules/servo_test/module.yaml`
- [x] 2.2 Add `SVT_GRP_A_MODE` (int32, 0–7, default 2) to `module.yaml` with enum values: 0=disabled, 1=DShot150, 2=DShot300, 3=DShot600, 4=PWM50, 5=PWM100, 6=PWM200, 7=PWM400
- [x] 2.3 Add `SVT_GRP_B_MODE` and `SVT_GRP_C_MODE` to `module.yaml` with same enum (default 0=disabled)
- [x] 2.4 Add `SVT_UAVCAN_EN` (bool, default false) to `module.yaml`
- [x] 2.5 Add `SVT_RAMP_EN` (bool, default true) to `module.yaml`
- [x] 2.6 Add `SVT_RAMP_TAU` (float, 0.1–5.0, default 0.5, unit=s) to `module.yaml`

## 3. ServoTest Header Rewrite

- [x] 3.1 Replace `_target_rpm` with `_target_throttle` (float 0.0–1.0) and `_output_throttle` (float, filtered output) in `src/modules/servo_test/ServoTest.hpp`
- [x] 3.2 Add state enum: `State { CONFIG, RUNNING, SETUP }` and `RampState { IDLE, STARTING, STOPPING }` to `ServoTest.hpp`
- [x] 3.3 Add `AlphaFilter<float> _ramp_filter` member; add `_ramp_state`, `_hold_start_time`, `_hold_progress` members
- [x] 3.4 Add `uORB::Subscription _battery_sub{ORB_ID(battery_status)}` for INA238 readings
- [x] 3.5 Update `DEFINE_PARAMETERS` block: remove old three params, add six new params
- [x] 3.6 Add private helpers: `void apply_group_mode(int group, int mode)`, `void publish_throttle(float value)`, `void enter_setup()`, `void exit_setup(bool commit)`

## 4. State Machine Implementation

- [x] 4.1 Implement CONFIG state in `ServoTest::Run()`: PUSH → transition to RUNNING (lock config, seed ramp filter to 0, set ramp state STARTING); K0 held → accumulate hold time, publish progress, transition to SETUP at 2.0 s
- [x] 4.2 Implement RUNNING state: dial delta → update `_target_throttle` (clamped 0–1); PUSH → set ramp state STOPPING; K0 → immediate `release_outputs()` → CONFIG
- [x] 4.3 Implement ramp state machine in `Run()`: STARTING uses `_ramp_filter.update(_target_throttle, dt)` until within 0.01, then switches to direct; STOPPING uses `_ramp_filter.update(0.0f, dt)` until < 0.01 then calls `release_outputs()` and transitions to CONFIG
- [x] 4.4 Implement SETUP state: dial scrolls 4-item menu; PUSH enters sub-page; K0 exits SETUP without commit
- [x] 4.5 Implement group protocol sub-page: dial scrolls 8 protocol options; PUSH calls `apply_group_mode(group, mode)` and returns to menu; K0 cancels and returns to menu

## 5. Protocol Application

- [x] 5.1 Implement `apply_group_mode(int group, int mode)` in `ServoTest.cpp`: map SVT_GRP_x_MODE enum to `PWM_MAIN_TIMx` value (-5/-4/-3/0/50/100/200/400), call `param_set()` and `param_notify_changes()`, then write channel min/max/fail/disabled for all channels in the group
- [x] 5.2 Implement DShot channel defaults (MIN=0, MAX=1999, DIS=0, FAIL=0) for `PWM_MAIN_MIN/MAX/DIS/FAIL` ch1–4 (group A), ch5–6 (group B), ch7–8 (group C)
- [x] 5.3 Implement PWM channel defaults (MIN=1100, MAX=1900, DIS=1100, FAIL=1100) for same channel sets
- [x] 5.4 Implement disabled group (set `PWM_MAIN_TIMx` = 0 and skip actuator_test publication for those channels)

## 6. Throttle Publication

- [x] 6.1 Rewrite `publish_motors()` → `publish_throttle(float value)`: iterate over the three groups; skip disabled groups; publish `actuator_test` for each active channel (Motor1–Motor4 for A, Motor5–Motor6 for B, Motor7–Motor8 for C) with `value` as throttle
- [x] 6.2 Add UAVCAN output path in `publish_throttle()`: when `SVT_UAVCAN_EN` is true, publish `actuator_test` for Motor1–8 (the UAVCAN driver will route these via `UAVCAN_EC_FUNC1-8`)
- [x] 6.3 Update `release_outputs()` to iterate over all three groups and release only enabled channels; also release UAVCAN channels if `SVT_UAVCAN_EN`

## 7. INA238 Display Integration

- [x] 7.1 Add `battery_status_s _battery{}` and `hrt_abstime _battery_time{0}` members; subscribe and copy in `Run()`
- [x] 7.2 Update `publish_display()` to write supply V+A into status bar field: "IN: XX.XV X.XA" when fresh, "IN: --V --A" when stale (>2 s) or never received

## 8. Display Layout Updates

- [x] 8.1 Update CONFIG display in `publish_display()`: centre area shows group protocol summary lines ("A: DShot300", "B: PWM 50Hz", "C: disabled", "CAN: N nodes"); lower line shows "hold K0 for setup"
- [x] 8.2 Update RUNNING display: centre shows throttle % (large numeric, `_output_throttle * 100`); units "%" ; status_text = "RUNNING"; ESC telemetry in info_line1/2/3 unchanged
- [x] 8.3 Implement SETUP menu display: list 4 items with highlight indicator; active sub-page shows protocol list with current selection
- [x] 8.4 Implement K0 hold progress bar: pass `_hold_progress` (0.0–1.0) in `display_command` (add field or reuse numeric_value during hold) and render as a fill bar in st7789_display or encode into a status line

## 9. Build Verification

- [x] 9.1 Verify `make crystal_nuc-h7xx_default` builds cleanly with no warnings on new params and includes
- [x] 9.2 Verify INA238 driver is present in the build output (`nm` or `size` check, or confirm Kconfig enables correctly)
- [x] 9.3 Confirm retired params (`SVT_RPM_STEP`, `SVT_RPM_MAX`, `SVT_NUM_MOT`) generate no `param not found` errors at boot
