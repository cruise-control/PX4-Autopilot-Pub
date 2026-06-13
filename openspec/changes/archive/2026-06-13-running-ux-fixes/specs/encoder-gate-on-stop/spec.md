## ADDED Requirements

### Requirement: Encoder delta ignored while stop button is held in RUNNING
In RUNNING state, when the main button is depressed (`push_rising`), the module SHALL set a `_run_push_held` flag and SHALL NOT apply any `enc.delta` to `_target_throttle` while the flag is set. The flag SHALL be cleared on `push_falling` in RUNNING state.

#### Scenario: No throttle change while stop button held
- **WHEN** the main button is pressed and held in RUNNING state and the encoder is rotated
- **THEN** `_target_throttle` remains unchanged during the button hold

#### Scenario: Throttle changes resume after button release
- **WHEN** the main button is released in RUNNING state (or the state has transitioned away)
- **THEN** subsequent encoder rotation updates `_target_throttle` normally

#### Scenario: Stop still initiates on push_rising
- **WHEN** the main button is pressed in RUNNING state
- **THEN** `stop_running(false)` is called immediately on `push_rising` (unchanged); the encoder gate is a side effect of that same event
