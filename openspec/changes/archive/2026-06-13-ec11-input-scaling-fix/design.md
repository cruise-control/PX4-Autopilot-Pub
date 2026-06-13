## Context

The EC11 encoder hardware produces 4 quadrature state transitions per physical detent (click). The driver's `process_encoder_step()` adds `step*4` to `_delta` on every transition, so one click produces `delta = ±16`. Consumers receive this raw inflated value.

In `servo_test`:
- Throttle: `enc.delta * 0.02f` → one click = ±0.32 (32%). Full 0–1 range in ~3 clicks.
- Menu: `(_menu_item + enc.delta) % 4` → `16 % 4 = 0` → menu item never advances.
- Sub-page: `(_sub_item + enc.delta) % 8` → `16 % 8 = 0` → same.

## Goals / Non-Goals

**Goals:**
- One physical detent = delta ±4 from the driver (remove the `*4` inflation)
- Throttle step per pulse tunable via `SVT_THR_STEP`, default gives ~1% per detent
- Menu navigation advances exactly one item per physical detent, reliably

**Non-Goals:**
- Velocity-sensitive throttle scaling
- Changing debounce or noise filtering in the driver
- Affecting `event.position` semantics for external consumers

## Decisions

### 1. Fix in the driver, not the consumer

**Decision:** Remove `*4` from `process_encoder_step` in the EC11 driver.

**Rationale:** The multiplier is incorrect at the source — each quadrature edge is ±1 and should be published as such. Any consumer that wants "detent units" should accumulate 4 pulses. Fixing it in the driver fixes all current and future consumers at once, rather than requiring every consumer to compensate.

**Impact on `_position`:** `_position` will now increment ±1 per edge (±4 per detent) instead of ±4 per edge (±16 per detent). No consumer currently uses `event.position` for anything beyond display, so this is safe.

### 2. Throttle step as a parameter (`SVT_THR_STEP`)

**Decision:** Replace the hardcoded `0.02f` multiplier with `_param_thr_step.get()`, defaulting to `0.0025f`.

**Rationale:** `0.0025` per pulse × 4 pulses per detent = 0.01 (1%) per click. Covering 0–100% takes 100 detents ≈ 5 full revolutions of the EC11. A parameter allows field tuning without recompile.

**Default derivation:**
```
target: ~1% per detent click
detent = 4 quadrature pulses (after driver fix)
step per pulse = 0.01 / 4 = 0.0025
```

### 3. Menu navigation via delta accumulator

**Decision:** Add `_menu_delta_acc` (int32) in `servo_test`. On each encoder event, add `enc.delta` to the accumulator. Advance the menu/sub-page selection by one step for each ±4 consumed, leaving the remainder in the accumulator.

```cpp
_menu_delta_acc += enc.delta;
while (_menu_delta_acc >=  4) { _menu_delta_acc -= 4; advance(+1); }
while (_menu_delta_acc <= -4) { _menu_delta_acc += 4; advance(-1); }
```

**Rationale:** This makes selection advance exactly once per physical detent, regardless of ISR timing jitter (which may occasionally produce delta=3 or delta=5 for a single detent). The accumulator absorbs partial events and fires cleanly on the next one.

**Reset:** `_menu_delta_acc` is reset to 0 on state transitions (CONFIG↔RUNNING↔SETUP) so accumulated partial events don't carry over.

## Risks / Trade-offs

**`_position` value change** → External tools reading `event.position` (e.g., via uORB listener) will see values 4× smaller after the fix. No flight-critical consumer exists on this platform; only a cosmetic change.

**Delta accumulator reset on state transition** → If the user is mid-turn while a state transition fires, they lose the partial detent. Acceptable — state transitions (button presses) and rotation are rarely simultaneous.

## Migration Plan

1. Remove `*4` from `EC11RotaryEncoder::process_encoder_step()`
2. Add `SVT_THR_STEP` to `module.yaml`
3. Update throttle delta application in `ServoTest::Run()` CONFIG and RUNNING cases
4. Add `_menu_delta_acc` member and accumulator logic for SETUP navigation
5. Reset `_menu_delta_acc` in `enter_setup()` and `exit_setup()`
6. Set `SVT_THR_STEP` default in `rc.board_defaults`
7. Build and verify

## Open Questions

None — root cause and fix are fully understood.
