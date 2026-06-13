## Context

`servo_test` currently drives up to N motors via a single `actuator_test` uORB publication with a hardwired DShot300 protocol and an RPM-target model. The Crystal NUC-H7xx board has three independent timer groups and a CAN bus, making it capable of simultaneously running DShot, PWM, and UAVCAN ESC outputs. The tester needs to expose all three protocol families through a unified throttle model and a simple encoder+button UI.

The board's output hardware:
- TIM1 (Group A, ch1–4): supports DShot150/300/600 and PWM; bidir DShot on ch1–4 only
- TIM4 (Group B, ch5–6): supports DShot150/300/600 and PWM; no bidir
- TIM2 (Group C, ch7–8): supports DShot150/300/600 and PWM; no bidir
- FDCAN1: UAVCAN/DroneCAN ESC commands, independent of timer groups

Protocol for each timer group is set via `PWM_MAIN_TIM{0,1,2}` params (negative = DShot rate, positive = PWM Hz). These params are read by `pwm_out` and `dshot` drivers at startup and on param update.

## Goals / Non-Goals

**Goals:**
- Three-state machine: CONFIG → RUNNING → CONFIG, with SETUP overlay accessible from CONFIG
- Normalized 0.0–1.0 throttle sent to all active channels each cycle
- Per-timer-group protocol selection persisted in `PWM_MAIN_TIM{0,1,2}` params
- Alpha-filter ramp on start/stop transitions; direct throttle during steady running
- INA238 I2C power monitor subscription via `battery_status` uORB
- UAVCAN ESC command output via existing DroneCAN infrastructure
- Min/max/fail/disabled per channel auto-set by protocol when group mode changes

**Non-Goals:**
- Per-channel independent throttle
- Automated test sequences
- Output voltage selection or switching
- Renaming the module

## Decisions

### 1. Throttle model: normalized float, not RPM

**Decision**: Replace `_target_rpm` with `_target_throttle` (0.0–1.0). The same value is sent to all active channels.

**Rationale**: RPM is ESC-firmware-specific, open-loop, and meaningless for PWM servo mode and UAVCAN. A normalized throttle maps cleanly to `actuator_test.value` (which the mixer already expects as 0–1), to UAVCAN ESC commands, and to the alpha-filter ramp. ESC telemetry from bidir DShot still provides actual RPM for display.

**Alternative considered**: Keep RPM as the setpoint and convert per-protocol. Rejected — the conversion requires knowing KV and voltage, which are device-specific and unavailable at the module level.

### 2. Protocol configuration via existing PWM_MAIN_TIM params

**Decision**: `SVT_GRP_{A,B,C}_MODE` stores the group protocol selection as an integer enum. On SETUP exit, the module writes `PWM_MAIN_TIM{0,1,2}` via `param_set` and triggers a param update so `pwm_out`/`dshot` drivers reconfigure without a reboot.

**Rationale**: PX4's output drivers already watch `parameter_update` and apply `PWM_MAIN_TIM*` changes at runtime (within the driver's next cycle). This avoids reimplementing protocol switching logic; the tester just sets the right param value.

**Mapping**:
```
SVT_GRP_x_MODE value → PWM_MAIN_TIMx value
  0 = disabled         →  0
  1 = DShot150         → -5
  2 = DShot300         → -4
  3 = DShot600         → -3
  4 = PWM 50 Hz        →  50
  5 = PWM 100 Hz       →  100
  6 = PWM 200 Hz       →  200
  7 = PWM 400 Hz       →  400
```

**Alternative considered**: Direct timer reconfiguration via ioctl. Rejected — fragile, bypasses PX4 param system, and breaks if driver internals change.

### 3. Min/max/fail/disabled auto-set on group mode change

**Decision**: When the user confirms a group protocol in SETUP, the module writes `PWM_MAIN_MIN/MAX/DIS/FAIL` for all channels in that group using protocol-appropriate defaults:
```
DShot:     MIN=0   MAX=1999  DIS=0    FAIL=0
PWM ESC:   MIN=1100 MAX=1900 DIS=1100 FAIL=1100
```

**Rationale**: AM32 ESC defaults use 1100–1900 µs range (servo_low/high_threshold). Writing these automatically prevents incorrect arming range on protocol switch.

### 4. Alpha filter on transitions only, direct throttle during running

**Decision**: `AlphaFilter<float>` is used only during start and stop ramp phases. During steady RUNNING the output equals `_target_throttle` directly.

**Rationale**: The filter is a safety/longevity feature for mechanical stress on motor start/stop. During running, the user expects immediate dial response. Applying the filter to all dial movements would make the knob feel laggy and unresponsive.

**Implementation**:
- Start: `_ramp_filter.reset(0.0f)`, target = `_target_throttle`, ramp until `|output - target| < 0.01f`
- Stop: `_ramp_filter.reset(_output_throttle)`, target = `0.0f`, ramp until `output < 0.01f` then release

### 5. UAVCAN output via uavcan_esc_setpoint publication

**Decision**: When `SVT_UAVCAN_EN` is set, the module publishes `actuator_test` for functions Motor1–8 as normal. The UAVCAN driver maps these to CAN ESC commands via the existing `UAVCAN_EC_FUNC1-8` function assignments already set in rc.board_defaults.

**Rationale**: Reusing `actuator_test` + the existing UAVCAN function mapping avoids a separate output path. The UAVCAN driver already handles the Motor1-8 → CAN ESC index mapping.

### 6. INA238 via battery_status subscription

**Decision**: `servo_test` subscribes to `ORB_ID(battery_status)` instance 0. The INA238 driver publishes to this topic. No direct I2C access from the module.

**Rationale**: Clean separation — the driver handles I2C timing and error recovery; the module just reads the last published value. If no INA238 is present, the topic is simply never updated and the display shows "---".

### 7. Hold detection in the module, not the encoder driver

**Decision**: The 2 s K0 hold is detected in `servo_test` by tracking `k0_rising` and `k0_falling` events and measuring elapsed time. A `_hold_progress` float (0.0–1.0) is published in `display_command` for the progress bar.

**Rationale**: The encoder driver publishes edge events and should not embed application-level timing policies.

## Risks / Trade-offs

**PWM_MAIN_TIM param write during SETUP could disrupt running output drivers** → Mitigation: param writes only occur on SETUP exit, which is only reachable from CONFIG state (output stopped). Drivers reconfigure without motor output.

**bidir DShot only on Group A (TIM1)** → Groups B and C in DShot mode will not provide ESC RPM telemetry via bidir. Serial DShot telemetry (DSHOT_TEL_CFG=102) still works for all groups. Documented in SETUP display.

**INA238 not present** → `battery_status` never updates; display shows "IN: --V --A". Non-fatal.

**UAVCAN ESC detection delay** → After boot, DroneCAN DNA allocation takes a few seconds. UAVCAN mode in SETUP should note "N nodes" from the live node list, which may show 0 at first. Non-fatal; user can see node count before starting.

**Alpha filter convergence time** → With tau=0.5 s, 99% convergence takes ~2.5 s. For a fast ramp-down e-stop this is bypassed (K0 releases immediately). For graceful stop, the 2.5 s ramp is acceptable for motor longevity.

## Migration Plan

1. Remove `SVT_RPM_STEP`, `SVT_RPM_MAX`, `SVT_NUM_MOT` from `module.yaml` and code
2. Add new params to `module.yaml`
3. Update `rc.board_defaults`: remove old SVT param sets, add INA238 start, add new SVT defaults
4. Update `default.px4board`: add `CONFIG_DRIVERS_POWER_MONITOR_INA238=y`
5. Rewrite `ServoTest.cpp/.hpp` with new state machine, throttle model, filter, display

No backwards-compatibility shim needed — this is a board-local module with no external consumers.

## Open Questions

- INA238 I2C bus number and address: to be confirmed from hardware schematic (default addr 0x45 if A0=VCC, 0x40 if A0=GND)
- PWM frequency options: 50/100/200/400 Hz covers AM32 range; confirm if other frequencies needed
- UAVCAN node scan UI: show count only ("2 nodes") or list node IDs? Count is sufficient for v1
