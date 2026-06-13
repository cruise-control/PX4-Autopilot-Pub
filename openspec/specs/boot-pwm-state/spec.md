## Requirements

### Requirement: All direct outputs boot in PWM 50Hz with valid pulse widths
On a factory-fresh board (no flash values), the following SHALL be set via `param set-default` in `rc.board_defaults`:
- `PWM_MAIN_TIM0/1/2 = 50` (PWM 50Hz on all three timer groups)
- `PWM_MAIN_FUNC1-8 = 101-108` (Motor1-8 mapped to all 8 channels)
- `PWM_MAIN_MIN1-8 = 1100`, `MAX1-8 = 1900`, `DIS1-8 = 1000`, `FAIL1-8 = 1000` (valid PWM µs values)
- `SVT_GRP_A_MODE = 4`, `SVT_GRP_B_MODE = 4`, `SVT_GRP_C_MODE = 4` (UI shows PWM 50Hz)

When flash values exist (e.g. from a prior SETUP commit or explicit GCS save), those flash values SHALL take precedence and the `param set-default` lines SHALL have no effect.

#### Scenario: Factory-fresh board first boot
- **WHEN** the board boots for the first time with no values stored in param flash
- **THEN** all 8 channels are mapped to Motor1-8 in PWM 50Hz mode with valid pulse widths

#### Scenario: Boot with operator-saved params
- **WHEN** the operator explicitly saved params (via GCS, MAVLink, or `param save`) and the board reboots
- **THEN** the saved values are retained; `param set-default` lines in rc.board_defaults have no effect

#### Scenario: Boot after a DShot SETUP commit
- **WHEN** a prior SETUP committed DShot300 on Group A (writing TIM0=-4 to flash) and the board reboots
- **THEN** `PWM_MAIN_TIM0` remains -4 (DShot300) on the next boot; the board does NOT reset to PWM 50Hz

#### Scenario: servo_test UI reflects actual flash state on boot
- **WHEN** the board boots with SVT_GRP_A_MODE stored in flash from a prior SETUP commit
- **THEN** the display shows the stored mode (e.g. DShot300), not the `param set-default` value

### Requirement: SETUP commit remains the only way to change the protocol within a session
The `apply_group_mode()` function SHALL continue to write protocol params when SETUP is committed. This is unchanged.

#### Scenario: DShot in the same boot session
- **WHEN** the operator runs SETUP and commits Group A = DShot300
- **THEN** `PWM_MAIN_TIM0 = -4` and Group A operates in DShot300 for the rest of that session
