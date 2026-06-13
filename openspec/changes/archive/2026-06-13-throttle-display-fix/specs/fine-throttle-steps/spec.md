## ADDED Requirements

### Requirement: Quarter-percent throttle resolution
The EC11 encoder SHALL change the normalized throttle (`_target_throttle`) by `SVT_THR_STEP` per raw quadrature pulse. The board default for `SVT_THR_STEP` SHALL be 0.000625, producing 0.25% (0.0025 normalized) per physical detent (4 pulses × 0.000625 = 0.0025).

#### Scenario: Single detent step in CONFIG
- **WHEN** the encoder is rotated one detent clockwise in CONFIG state
- **THEN** `_target_throttle` increases by exactly 0.0025 (0.25%)

#### Scenario: Single detent step in RUNNING
- **WHEN** the encoder is rotated one detent clockwise in RUNNING state
- **THEN** `_target_throttle` increases by exactly 0.0025 (0.25%)

#### Scenario: Throttle clamped at bounds
- **WHEN** `_target_throttle` would exceed 1.0 or go below 0.0
- **THEN** it is clamped to 1.0 or 0.0 respectively
