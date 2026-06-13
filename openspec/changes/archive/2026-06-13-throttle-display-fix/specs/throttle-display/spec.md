## ADDED Requirements

### Requirement: Two-decimal throttle display
The `display_command` `numeric_value` field for the throttle readout SHALL be set to `decimal_places = 2` in both CONFIG and RUNNING states, so that 0.25% increments are legible (e.g., "0.25 % SET", "12.50 %").

#### Scenario: Sub-percent value visible in CONFIG
- **WHEN** `_target_throttle` is 0.0025 (one detent from zero) in CONFIG state
- **THEN** the display shows "0.25" with units "% SET"

#### Scenario: Sub-percent value visible in RUNNING
- **WHEN** `_output_throttle` is 0.0025 in RUNNING/IDLE state
- **THEN** the display shows "0.25" with units "%"

### Requirement: Stable throttle display on start transition
During the STARTING ramp, the displayed numeric value SHALL reflect `_target_throttle` (the commanded setpoint) rather than `_output_throttle` (the ramping output), so the screen value does not visibly drop to 0% when the user presses PUSH to start.

#### Scenario: No display jump on start button press
- **WHEN** `_target_throttle` is 0.50 in CONFIG and the user presses PUSH
- **THEN** the display continues to show "50.00 %" during the STARTING ramp (not "0.00 %")

#### Scenario: IDLE and STOPPING states show output throttle
- **WHEN** `_ramp_state` is IDLE or STOPPING
- **THEN** the display shows `_output_throttle * 100.0f` (actual output value)

### Requirement: Throttle reset to zero on stop
When a stop completes (graceful STOPPING ramp or immediate e-stop), `_target_throttle` SHALL be reset to 0.0f so that the CONFIG state display shows 0% rather than the previous setpoint.

#### Scenario: CONFIG shows 0% after graceful stop
- **WHEN** the STOPPING ramp completes and state returns to CONFIG
- **THEN** the displayed throttle is 0.00 % SET

#### Scenario: CONFIG shows 0% after e-stop
- **WHEN** K0 triggers an immediate stop from RUNNING
- **THEN** `_target_throttle` is 0.0f and CONFIG display shows 0.00 % SET
