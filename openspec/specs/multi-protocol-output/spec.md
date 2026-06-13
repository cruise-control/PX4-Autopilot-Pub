## ADDED Requirements

### Requirement: Timer group protocol selection
The system SHALL support independent protocol selection for each of three timer output groups: Group A (channels 1–4, TIM1), Group B (channels 5–6, TIM4), and Group C (channels 7–8, TIM2). Each group SHALL be independently configurable as DShot150, DShot300, DShot600, PWM at 50/100/200/400 Hz, or disabled. Protocol selection SHALL be persisted in `PWM_MAIN_TIM0`, `PWM_MAIN_TIM1`, and `PWM_MAIN_TIM2` parameters respectively.

#### Scenario: Group A set to DShot300
- **WHEN** the user selects DShot300 for Group A in SETUP and confirms
- **THEN** `PWM_MAIN_TIM0` is set to -4 and a parameter update is triggered so the dshot driver reconfigures Group A channels

#### Scenario: Group B set to PWM 50 Hz
- **WHEN** the user selects PWM 50 Hz for Group B in SETUP and confirms
- **THEN** `PWM_MAIN_TIM1` is set to 50 and a parameter update is triggered so the pwm_out driver reconfigures Group B channels

#### Scenario: Group disabled
- **WHEN** the user selects disabled for a group in SETUP and confirms
- **THEN** the corresponding `PWM_MAIN_TIMx` is set to 0 and no actuator_test publications are made for channels in that group during RUNNING

### Requirement: Normalized throttle to all active channels
The system SHALL apply a single normalized throttle value (0.0–1.0) uniformly to all active channels each control cycle. An active channel is any channel in a group whose protocol is not disabled, plus any UAVCAN output when enabled.

#### Scenario: Single throttle drives multiple groups
- **WHEN** Group A is DShot300 (channels 1–4), Group B is PWM 50 Hz (channels 5–6), and UAVCAN is enabled
- **THEN** each 20 ms cycle publishes actuator_test for Motor1–Motor6 and a UAVCAN ESC command, all with the same normalized throttle value

#### Scenario: Zero throttle on disabled group
- **WHEN** Group C is disabled
- **THEN** no actuator_test publications are made for channels 7–8 regardless of throttle value

### Requirement: Auto-configure channel min/max/fail/disabled on protocol change
When the user confirms a protocol selection for a group in SETUP, the system SHALL automatically write appropriate `PWM_MAIN_MIN/MAX/DIS/FAIL` values for all channels in that group based on the selected protocol.

#### Scenario: DShot group channel limits
- **WHEN** a group is set to any DShot rate
- **THEN** MIN=0, MAX=1999, DIS=0, FAIL=0 are written for all channels in that group

#### Scenario: PWM group channel limits
- **WHEN** a group is set to any PWM frequency
- **THEN** MIN=1100, MAX=1900, DIS=1100, FAIL=1100 are written for all channels in that group (AM32 servo_low/high_threshold defaults)

### Requirement: UAVCAN ESC output
The system SHALL support UAVCAN/DroneCAN ESC output as an independent output mode, controlled by the `SVT_UAVCAN_EN` parameter. When enabled, the normalized throttle SHALL be sent to detected DroneCAN ESC nodes via the existing `UAVCAN_EC_FUNC1-8` function mapping.

#### Scenario: UAVCAN enabled with detected nodes
- **WHEN** `SVT_UAVCAN_EN` is true and DroneCAN ESC nodes are present on CAN1
- **THEN** each control cycle publishes actuator_test for Motor1–8 which the UAVCAN driver maps to CAN ESC commands

#### Scenario: UAVCAN disabled
- **WHEN** `SVT_UAVCAN_EN` is false
- **THEN** no UAVCAN-specific output occurs; timer group outputs are unaffected

### Requirement: Config locked during RUNNING
The system SHALL prevent protocol changes to any group while in RUNNING state. Group protocol, UAVCAN enable, and all channel min/max/fail/disabled values SHALL be read-only during RUNNING.

#### Scenario: Protocol selection blocked during RUNNING
- **WHEN** the system is in RUNNING state
- **THEN** the SETUP menu SHALL be inaccessible and K0 hold SHALL act as e-stop only
