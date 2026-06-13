## Requirements

### Requirement: SETUP commit writes PWM_MAIN_FUNC for all channels in the group
When `apply_group_mode(group, mode)` is called, it SHALL write `PWM_MAIN_FUNC{ch}` for every channel `ch` in the group's range. The value SHALL be `100 + ch` (Motor function) when `mode != 0`, and `0` (disabled) when `mode == 0`.

#### Scenario: Group A committed with DShot300
- **WHEN** SETUP commits Group A (channels 1–4) with mode = DShot300
- **THEN** `PWM_MAIN_FUNC1 = 101`, `PWM_MAIN_FUNC2 = 102`, `PWM_MAIN_FUNC3 = 103`, `PWM_MAIN_FUNC4 = 104`

#### Scenario: Group B committed as disabled
- **WHEN** SETUP commits Group B (channels 5–6) with mode = disabled (0)
- **THEN** `PWM_MAIN_FUNC5 = 0` and `PWM_MAIN_FUNC6 = 0`

#### Scenario: Group C committed with PWM 50Hz
- **WHEN** SETUP commits Group C (channels 7–8) with mode = PWM 50Hz
- **THEN** `PWM_MAIN_FUNC7 = 107` and `PWM_MAIN_FUNC8 = 108`

### Requirement: Disabled group channels have FUNC zeroed alongside protocol zeroed
When `mode == 0`, `apply_group_mode()` SHALL write `PWM_MAIN_TIM{group} = 0` AND `PWM_MAIN_FUNC{ch} = 0` for all channels in the group, and SHALL also update the `SVT_GRP_{X}_MODE` param. MIN/MAX/DIS/FAIL writes are not required for disabled channels.

#### Scenario: Disabling a previously enabled group
- **WHEN** Group A was previously DShot300 (FUNC1-4 = 101-104) and SETUP commits it as disabled
- **THEN** `PWM_MAIN_TIM0 = 0` and `PWM_MAIN_FUNC1-4 = 0`

### Requirement: Boot-time FUNC state
The boot-time state of `PWM_MAIN_FUNC1-8` is defined by the `boot-pwm-state` capability. On every boot, `rc.board_defaults` sets all channels to Motor1–8 (101–108) via `param set`, overriding any SETUP-committed values from flash. SETUP commits then override these per-group within the same session.
