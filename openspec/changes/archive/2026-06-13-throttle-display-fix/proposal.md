## Why

The motor tester's throttle steps at 1% per detent (too coarse for precise ESC characterisation), the display doesn't render sub-1% values, and pressing the main button causes jarring throttle jumps on screen due to `_output_throttle` being reset to 0 on start and the old `_target_throttle` being shown on stop completion.

## What Changes

- Reduce `SVT_THR_STEP` default from 0.0025 → 0.000625 (0.25% per detent instead of 1%)
- Update the throttle display to `decimal_places = 2` so 0.25% increments are visible
- Fix STARTING ramp display: show `_target_throttle` (not `_output_throttle`) during ramp-up so the screen value doesn't flash to 0% on button press
- Reset `_target_throttle = 0.0f` when a stop completes (graceful or immediate) so CONFIG screen shows 0 after a stop rather than the previous set-point

## Capabilities

### New Capabilities

- `fine-throttle-steps`: Throttle adjustable in 0.25% per detent increments with two-decimal display

### Modified Capabilities

- `throttle-display`: Display now shows two decimal places and is free of jump artefacts across state transitions

## Impact

- `src/modules/servo_test/module.yaml`: `SVT_THR_STEP` default
- `boards/crystal/nuc-h7xx/init/rc.board_defaults`: `SVT_THR_STEP` board default
- `src/modules/servo_test/ServoTest.cpp`: `publish_display()` decimal places and RUNNING numeric source; `stop_running()` and STOPPING completion path reset `_target_throttle`

## Non-goals

- Per-channel throttle control (all channels remain locked together)
- Variable step size from the UI (param change only)
- ESC RPM display changes
