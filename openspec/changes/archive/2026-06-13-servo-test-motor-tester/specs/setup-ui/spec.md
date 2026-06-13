## ADDED Requirements

### Requirement: Three-state machine (CONFIG / RUNNING / SETUP overlay)
The system SHALL operate in three states: CONFIG (idle, outputs stopped), RUNNING (outputs active, config locked), and SETUP (protocol configuration overlay, accessible only from CONFIG). State transitions SHALL be: CONFIG → RUNNING via PUSH; RUNNING → CONFIG via PUSH (graceful) or K0 (e-stop); CONFIG → SETUP via K0 hold; SETUP → CONFIG via K0 or completing configuration.

#### Scenario: PUSH starts output from CONFIG
- **WHEN** in CONFIG state and the user presses PUSH
- **THEN** the system transitions to RUNNING, locks group configuration, and begins publishing throttle to active channels

#### Scenario: PUSH stops output gracefully from RUNNING
- **WHEN** in RUNNING state and the user presses PUSH
- **THEN** the system begins a graceful ramp-down (if SVT_RAMP_EN) and transitions to CONFIG after outputs are released

#### Scenario: K0 e-stops from RUNNING
- **WHEN** in RUNNING state and the user presses K0
- **THEN** actuator RELEASE_CONTROL is published immediately and the system transitions to CONFIG

#### Scenario: SETUP inaccessible from RUNNING
- **WHEN** in RUNNING state and the user holds K0
- **THEN** K0 acts as e-stop; SETUP is NOT entered

### Requirement: Hold K0 to enter SETUP with visual progress
From CONFIG state, the system SHALL enter SETUP when K0 is held continuously for 2.0 seconds. A visual progress indicator SHALL be shown on the display during the hold, filling from 0% to 100% over the 2 s duration. If K0 is released before 2.0 s, the hold is cancelled and the display returns to normal CONFIG state.

#### Scenario: Successful 2 s hold enters SETUP
- **WHEN** in CONFIG state and K0 is held for 2.0 s without release
- **THEN** the system transitions to SETUP state and shows the protocol selection menu

#### Scenario: Partial hold cancelled
- **WHEN** in CONFIG state and K0 is held for less than 2.0 s then released
- **THEN** SETUP is not entered and the display returns to CONFIG layout

#### Scenario: Progress bar fills during hold
- **WHEN** K0 is being held in CONFIG state
- **THEN** the display shows a progress bar that fills proportionally to the elapsed hold time relative to 2.0 s

### Requirement: SETUP menu navigation
The SETUP menu SHALL present four items: Group A protocol, Group B protocol, Group C protocol, and UAVCAN enable. The dial SHALL scroll the highlight through items. PUSH SHALL enter the highlighted item's sub-page. K0 SHALL exit SETUP and return to CONFIG, discarding any uncommitted changes.

#### Scenario: Dial scrolls through menu items
- **WHEN** in SETUP and the user turns the dial
- **THEN** the highlight moves through the four menu items in order

#### Scenario: PUSH enters sub-page
- **WHEN** a menu item is highlighted and the user presses PUSH
- **THEN** the display shows the sub-page for that item (protocol choices or UAVCAN toggle)

#### Scenario: K0 exits SETUP without saving
- **WHEN** in SETUP and the user presses K0
- **THEN** the system returns to CONFIG without changing any group protocol params

### Requirement: Protocol sub-page selection
Each group sub-page SHALL list protocol options: DShot150, DShot300, DShot600, PWM 50 Hz, PWM 100 Hz, PWM 200 Hz, PWM 400 Hz, disabled. The dial SHALL scroll the selection. PUSH SHALL confirm the selection, write the corresponding `PWM_MAIN_TIMx` and channel min/max/fail/disabled params, and return to the SETUP menu. K0 SHALL cancel and return to the SETUP menu without writing params.

#### Scenario: User selects DShot600 for Group A
- **WHEN** in Group A sub-page and the user scrolls to DShot600 and presses PUSH
- **THEN** `PWM_MAIN_TIM0` is set to -3, channel 1–4 min/max/fail/disabled are set for DShot, and the SETUP menu is shown with Group A displaying "DShot600"

#### Scenario: Cancel sub-page with K0
- **WHEN** in a group sub-page and the user presses K0
- **THEN** no params are changed and the SETUP menu is shown with the previous protocol unchanged

### Requirement: CONFIG state display layout
In CONFIG state, the display SHALL show: status label "CONFIG" and live supply V+A in the top bar; active group protocol summary (Group A/B/C and UAVCAN) in the centre area; and a prompt "hold K0 for setup" in the lower area.

#### Scenario: CONFIG display content
- **WHEN** in CONFIG state
- **THEN** display shows group protocols for A, B, C and UAVCAN node count, plus supply readings and hold prompt

### Requirement: RUNNING state display layout
In RUNNING state, the display SHALL show: status label "RUNNING" and live supply V+A in the top bar; current throttle percentage in the centre area; and ESC telemetry (RPM, voltage, current, temperature from esc_status) in the lower telemetry band.

#### Scenario: RUNNING display content
- **WHEN** in RUNNING state
- **THEN** display shows throttle % large in centre, ESC telemetry in lower band, supply V+A and "RUNNING" in top bar
