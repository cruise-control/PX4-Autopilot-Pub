## ADDED Requirements

### Requirement: Alpha-filter ramp on output enable
When signal ramping is enabled (`SVT_RAMP_EN` = true), the system SHALL apply a first-order alpha filter (low-pass) to the throttle output during the start transition (CONFIG → RUNNING). The filter SHALL be initialized to 0.0 and track the current dial target, converging with time constant `SVT_RAMP_TAU` seconds. Direct throttle control SHALL resume once the filtered output is within 0.01 of the target.

#### Scenario: Ramp from zero to target on enable
- **WHEN** `SVT_RAMP_EN` is true, `SVT_RAMP_TAU` is 0.5 s, and the user starts the output with dial at 0.5
- **THEN** the published throttle begins at ~0.0 and converges exponentially toward 0.5, reaching ~63% (0.315) after 0.5 s and ~95% (0.475) after 1.5 s

#### Scenario: Ramp disabled — immediate full throttle
- **WHEN** `SVT_RAMP_EN` is false and the user starts the output with dial at 0.5
- **THEN** the published throttle is 0.5 from the first cycle

#### Scenario: Dial unchanged during start ramp
- **WHEN** a start ramp is in progress
- **THEN** the filter target is the current dial position; the output converges toward it smoothly

### Requirement: Alpha-filter ramp on graceful output disable
When signal ramping is enabled, the system SHALL apply the alpha filter to ramp the throttle to 0.0 during the graceful stop transition (PUSH while RUNNING). The filter SHALL be initialized to the current output throttle and track a target of 0.0. Actuator outputs SHALL be released only after the filtered output falls below 0.01.

#### Scenario: Graceful ramp-down to zero
- **WHEN** `SVT_RAMP_EN` is true, current output throttle is 0.8, and the user presses PUSH
- **THEN** the published throttle ramps exponentially from 0.8 toward 0.0 with time constant `SVT_RAMP_TAU`, and actuator RELEASE_CONTROL is published only after output < 0.01

#### Scenario: E-stop bypasses ramp
- **WHEN** K0 is pressed during RUNNING
- **THEN** actuator RELEASE_CONTROL is published immediately on the same cycle, regardless of `SVT_RAMP_EN`

### Requirement: Direct throttle during steady running
During steady RUNNING (after start ramp completes, before a stop is commanded), the system SHALL apply the dial position directly to the output throttle without filtering.

#### Scenario: Immediate dial response during running
- **WHEN** the system is in steady RUNNING state and the user turns the dial
- **THEN** the output throttle changes to match the new dial position within one control cycle (20 ms)

### Requirement: Ramp parameters
The system SHALL expose `SVT_RAMP_EN` (bool, default true) to enable/disable ramping and `SVT_RAMP_TAU` (float, seconds, default 0.5, range 0.1–5.0) to set the filter time constant. Both SHALL be readable and updatable via the standard PX4 parameter system.

#### Scenario: Parameter update during CONFIG
- **WHEN** `SVT_RAMP_TAU` is changed via param set while in CONFIG state
- **THEN** the new tau is applied to the next start or stop transition
