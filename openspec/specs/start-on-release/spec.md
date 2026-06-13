## ADDED Requirements

### Requirement: Test starts on button release, not button press
In CONFIG state, the servo_test module SHALL NOT call `start_running()` on `push_rising`. Instead it SHALL set an internal `_push_pending` flag on `push_rising` and call `start_running()` only on the subsequent `push_falling` event while `_push_pending` is true.

#### Scenario: Normal press-and-release starts the test
- **WHEN** the operator presses and fully releases the main button while in CONFIG state
- **THEN** `start_running()` is called on the falling edge and the module transitions to RUNNING

#### Scenario: Holding the button does not start the test
- **WHEN** the operator presses the main button and holds it without releasing
- **THEN** no state transition occurs while the button remains depressed

#### Scenario: Aborting a press by pressing K0 while button held
- **WHEN** the operator presses the main button (setting `_push_pending`) and then presses K0 before releasing the main button
- **THEN** `_push_pending` is cleared and `start_running()` is NOT called when the main button is eventually released

#### Scenario: Orphaned push_falling ignored if not pending
- **WHEN** `push_falling` arrives but `_push_pending` is false (e.g., button was held across a state transition)
- **THEN** no action is taken

### Requirement: Stop trigger is unchanged
In RUNNING state, pressing the main button (`push_rising`) SHALL still initiate a graceful stop immediately. The press-and-release requirement applies only to the CONFIG → RUNNING transition.

#### Scenario: Stop fires on push_rising in RUNNING
- **WHEN** the operator presses the main button while in RUNNING state
- **THEN** `stop_running(false)` is called immediately on the rising edge

### Requirement: _push_pending cleared on state exit
The `_push_pending` flag SHALL be cleared whenever the module leaves CONFIG state (entering RUNNING or SETUP) so that a stale pending state cannot trigger a start after returning to CONFIG.

#### Scenario: _push_pending cleared on entering SETUP
- **WHEN** the K0 hold completes and the module enters SETUP state
- **THEN** `_push_pending` is false in CONFIG on return from SETUP
