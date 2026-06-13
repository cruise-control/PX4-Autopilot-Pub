## Why

The test currently starts on `push_rising` (button press-down), so any accidental contact or bounce can fire the motors unintentionally. Requiring a full press-and-release (rising edge followed by falling edge) gives the operator a chance to abort by holding the button down, and provides a clearer intent signal before motors energise.

## What Changes

- Add a `_push_pending` flag to `ServoTest` that is set on `push_rising` and consumed on `push_falling` in CONFIG state
- Move `start_running()` call from `push_rising` to `push_falling` (only when `_push_pending` is set)
- Cancel `_push_pending` if K0 is pressed while the button is held (e.g., user decides to enter SETUP instead)
- Stop (RUNNING → CONFIG) retains its existing `push_rising` trigger — only the **start** path changes

## Capabilities

### New Capabilities

- `start-on-release`: Test output starts only after a complete press-and-release of the main button in CONFIG state

### Modified Capabilities

(none — stop behaviour is unchanged)

## Impact

- `src/modules/servo_test/ServoTest.hpp`: add `bool _push_pending{false}` member
- `src/modules/servo_test/ServoTest.cpp` `Run()` CONFIG encoder block: move `start_running()` from `push_rising` to `push_falling`; set/clear `_push_pending`; cancel on `k0_rising`

## Non-goals

- Changing the stop trigger (RUNNING still stops on `push_rising`)
- Double-click or long-press sequences for start
- Visual feedback during the button-held interval (button hold feedback already exists for K0; no equivalent for PUSH)
