## MODIFIED Requirements

### Requirement: Throttle reset to zero on stop
When a **graceful** stop completes (STOPPING ramp reaches zero), `_target_throttle` SHALL be preserved at its last-set value so the operator can restart at the same throttle without re-dialling. When an **immediate** stop occurs (e-stop via K0, or ramp disabled), `_target_throttle` SHALL be reset to 0.0f.

#### Scenario: CONFIG shows previous setpoint after graceful stop
- **WHEN** the STOPPING ramp completes and state returns to CONFIG
- **THEN** `_target_throttle` is unchanged from the value set while RUNNING, and the display shows that value

#### Scenario: CONFIG shows 0% after e-stop
- **WHEN** K0 triggers an immediate stop from RUNNING
- **THEN** `_target_throttle` is 0.0f and CONFIG display shows 0.00 % SET

#### Scenario: CONFIG shows 0% after stop with ramp disabled
- **WHEN** `SVT_RAMP_EN` is 0 and the stop button is pressed
- **THEN** `stop_running(immediate=true)` resets `_target_throttle` to 0.0f
