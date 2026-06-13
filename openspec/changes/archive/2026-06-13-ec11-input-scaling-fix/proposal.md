## Why

The EC11 driver multiplies every raw quadrature step by 4 (`_delta += step*4`), producing `delta = ±16` per physical detent instead of `±4`. This makes the throttle control in `servo_test` jump ~32% per click and makes the SETUP menu navigation non-functional (16 % 4 = 0 — the menu item never advances).

## What Changes

- **Fix** `EC11RotaryEncoder::process_encoder_step`: remove `*4` so each quadrature edge contributes ±1 to delta; one physical detent = delta ±4
- **Add** `SVT_THR_STEP` parameter (float, throttle change per delta unit, default 0.0025 = 0.25% per quadrature pulse = 1% per detent)
- **Replace** direct `enc.delta` use in `servo_test` throttle path with `enc.delta * _param_thr_step.get()`
- **Add** delta accumulator in `servo_test` for menu/sub-page navigation: advance selection only when `|accumulated_delta| >= 4` (one physical detent), consume in steps of ±4

## Capabilities

### New Capabilities

- `encoder-input-scaling`: Correct per-pulse delta from the EC11 driver and expose a tunable throttle-step param so consumers get predictable 1-click resolution

### Modified Capabilities

- `multi-protocol-output`: throttle dial step per click changes (requirement: dial SHALL resolve throttle in steps finer than 1%, parameterised)
- `setup-ui`: menu navigation SHALL advance exactly one item per physical detent (requirement: dial SHALL not skip or repeat items)

## Impact

- `src/drivers/ec11_rotary_encoder/EC11RotaryEncoder.cpp` — `process_encoder_step()`
- `src/modules/servo_test/ServoTest.hpp/.cpp` — throttle step, menu delta accumulator
- `src/modules/servo_test/module.yaml` — new `SVT_THR_STEP` param
- `boards/crystal/nuc-h7xx/init/rc.board_defaults` — set `SVT_THR_STEP` default

## Non-goals

- Changing the EC11 driver's `_position` semantics beyond the delta fix
- Per-axis acceleration or velocity-based throttle scaling
- Fixing any other encoder hardware concerns (debounce, noise)
