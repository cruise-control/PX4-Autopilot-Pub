## Why

The existing `servo_test` module drives motors via a single hardwired DShot300 output with a fixed RPM knob. The Crystal NUC-H7xx board is intended as a bench ESC tester supporting multiple output protocols (DShot, PWM, UAVCAN) and multiple ESC types simultaneously. The module needs to grow into that role.

## What Changes

- **Retire** `SVT_RPM_STEP`, `SVT_RPM_MAX`, `SVT_NUM_MOT` parameters
- **Add** per-timer-group protocol selection: Group A (ch1–4/TIM1), Group B (ch5–6/TIM4), Group C (ch7–8/TIM2) — each independently set to DShot150/300/600, PWM at selectable Hz, or disabled
- **Add** UAVCAN output mode: normalized throttle sent to detected DroneCAN ESC nodes over CAN1
- **Add** normalized 0.0–1.0 throttle model replacing RPM target; same throttle sent to all active channels
- **Add** alpha-filter signal ramping on start/stop transitions only (`SVT_RAMP_EN`, `SVT_RAMP_TAU`); dial adjustments remain direct during steady running
- **Add** CONFIG / RUNNING / SETUP state machine: hold K0 2 s (visual progress bar) to enter SETUP; PUSH to start/stop; K0 e-stop
- **Add** INA238 I2C power monitor integration: supply voltage + current displayed in status bar (up to 85 V range)
- **Add** SETUP UI: 4-item menu (Group A, Group B, Group C, UAVCAN) navigated by dial; PUSH confirms, K0 backs out
- **Update** display layout: status bar shows supply V+A (INA238); centre shows throttle %; telemetry band shows ESC RPM/V/A/temp

## Capabilities

### New Capabilities

- `multi-protocol-output`: Per-timer-group protocol selection (DShot150/300/600, PWM Hz, disabled) and UAVCAN output, all active simultaneously with a shared normalized throttle
- `signal-ramping`: Alpha-filter ramp on output enable/disable transitions; tau and enable/disable param-controlled
- `supply-monitoring`: INA238 I2C power monitor providing live supply voltage and current to the display
- `setup-ui`: Hold-to-enter SETUP state with protocol selection menu, 2 s K0 hold with visual progress, dial+button navigation

### Modified Capabilities

_(none — no existing specs)_

## Impact

- **Module**: `src/modules/servo_test/` — `ServoTest.cpp/.hpp`, `module.yaml`, `CMakeLists.txt`
- **Board config**: `boards/crystal/nuc-h7xx/default.px4board` — add `CONFIG_DRIVERS_POWER_MONITOR_INA238=y`
- **rc.board_defaults**: retire old SVT params; add INA238 start command; add new SVT group/ramp params
- **Params**: 6 new (`SVT_GRP_A_MODE`, `SVT_GRP_B_MODE`, `SVT_GRP_C_MODE`, `SVT_UAVCAN_EN`, `SVT_RAMP_EN`, `SVT_RAMP_TAU`); 3 retired
- **uORB**: new subscription to `battery_status` (INA238); new UAVCAN ESC command publication path
- **No change** to ec11_rotary_encoder or st7789_display drivers

## Non-goals

- Automated test sequences
- Output voltage selection or switching (user supplies voltage externally)
- Per-channel independent throttle control (all active channels share one throttle)
- Renaming the module (deferred)
