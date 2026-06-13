## ADDED Requirements

### Requirement: Driver reports one delta unit per quadrature edge
The EC11 driver SHALL add ±1 (not ±4) to `_delta` for each valid quadrature state transition. One physical detent (4 transitions) SHALL produce `event.delta = ±4`.

#### Scenario: Single detent CW produces delta +4
- **WHEN** the encoder is rotated one detent clockwise
- **THEN** the published `rotary_encoder_event_s.delta` is +4

#### Scenario: Single detent CCW produces delta -4
- **WHEN** the encoder is rotated one detent counter-clockwise
- **THEN** the published `rotary_encoder_event_s.delta` is -4

#### Scenario: Partial rotation produces partial delta
- **WHEN** the encoder moves 2 quadrature edges without completing a detent
- **THEN** the accumulated delta reflects ±2, published on the next work cycle

### Requirement: Tunable throttle step per pulse
`servo_test` SHALL expose `SVT_THR_STEP` (float, range 0.0001–0.02, default 0.0025) controlling the throttle change per quadrature pulse. One physical detent SHALL change throttle by `4 × SVT_THR_STEP`.

#### Scenario: Default step gives ~1% per detent
- **WHEN** `SVT_THR_STEP` is 0.0025 and the encoder is rotated one detent CW
- **THEN** `_target_throttle` increases by 0.01 (1%)

#### Scenario: Step is clamped to throttle range
- **WHEN** `_target_throttle` is 0.99 and the encoder moves one detent CW
- **THEN** `_target_throttle` is clamped to 1.0, not 1.01

#### Scenario: Parameter update takes effect immediately
- **WHEN** `SVT_THR_STEP` is changed via param set while in CONFIG state
- **THEN** the next encoder rotation uses the new step value
