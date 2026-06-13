## ADDED Requirements

### Requirement: INA238 supply voltage and current display
The system SHALL subscribe to `battery_status` uORB topic (instance 0) populated by the INA238 I2C power monitor driver and display the measured supply voltage and current in the display status bar at all times (CONFIG, SETUP, and RUNNING states).

#### Scenario: INA238 present and publishing
- **WHEN** the INA238 driver is running and publishing to battery_status
- **THEN** the display status bar shows "IN: XX.XV  X.XA" updated each time battery_status is received

#### Scenario: INA238 absent or not yet publishing
- **WHEN** no battery_status message has been received
- **THEN** the display status bar shows "IN: --V --A" without error or crash

### Requirement: INA238 driver enabled on board
The board configuration SHALL include `CONFIG_DRIVERS_POWER_MONITOR_INA238=y` and `rc.board_defaults` SHALL start the INA238 driver on the appropriate I2C bus and address.

#### Scenario: Driver starts at boot
- **WHEN** the board boots
- **THEN** the INA238 driver starts, initializes successfully, and begins publishing battery_status at its configured rate

### Requirement: Supply data shown in all states
The supply voltage and current reading SHALL appear in the display in CONFIG, SETUP, and RUNNING states. It SHALL NOT be hidden or replaced by other content during any state transition.

#### Scenario: Voltage visible during RUNNING
- **WHEN** the system is in RUNNING state with a motor spinning
- **THEN** the status bar continues to show live supply V and A alongside the RUNNING indicator
