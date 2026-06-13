## ADDED Requirements

### Requirement: All direct outputs boot in PWM 50Hz with valid pulse widths
On every boot, the following SHALL be set via `param set` (unconditional override) in `rc.board_defaults`:
- `PWM_MAIN_TIM0/1/2 = 50` (PWM 50Hz on all three timer groups)
- `PWM_MAIN_FUNC1-8 = 101-108` (Motor1-8 mapped to all 8 channels)
- `PWM_MAIN_MIN1-8 = 1100`, `MAX1-8 = 1900`, `DIS1-8 = 1000`, `FAIL1-8 = 1000` (valid PWM µs values)
- `SVT_GRP_A_MODE = 4`, `SVT_GRP_B_MODE = 4`, `SVT_GRP_C_MODE = 4` (UI shows PWM 50Hz)

#### Scenario: Boot after a DShot SETUP commit
- **WHEN** a prior SETUP committed DShot300 on Group A (writing TIM0=-4, MIN1-4=0, DIS1-4=0 to flash)
- **THEN** on next boot, `PWM_MAIN_TIM0 = 50`, `PWM_MAIN_DIS1-4 = 1000` are forced overriding flash, and ESCs receive valid 50Hz PWM signals

#### Scenario: Fresh board first boot
- **WHEN** the board boots for the first time with factory param defaults
- **THEN** all 8 channels are mapped to Motor1-8 in PWM 50Hz mode with valid pulse widths — identical to post-DShot-SETUP boot

#### Scenario: servo_test UI shows PWM 50Hz on boot
- **WHEN** the board boots and servo_test starts in CONFIG state
- **THEN** the display shows all three groups as "PWM 50Hz" (SVT_GRP_A/B/C_MODE = 4)

### Requirement: SETUP commit remains the only way to change the protocol
The `apply_group_mode()` function SHALL continue to write protocol params when SETUP is committed. The boot-time override does not prevent the operator from switching to DShot or other protocols within the same session via SETUP; it only ensures the session starts in a known PWM state.

#### Scenario: DShot in the same boot session
- **WHEN** the operator runs SETUP and commits Group A = DShot300
- **THEN** `PWM_MAIN_TIM0 = -4` and Group A operates in DShot300 for the rest of that session
- **AND** on the next reboot, `PWM_MAIN_TIM0` is reset to 50 (PWM 50Hz) by `rc.board_defaults`
