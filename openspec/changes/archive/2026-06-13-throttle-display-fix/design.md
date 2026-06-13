## Context

`servo_test` runs at 50 Hz and drives a normalized 0–1 throttle to all active output channels. The EC11 encoder feeds `delta` pulses (4 per physical detent) into `_target_throttle`. The ST7789 display receives a `display_command` message with `numeric_value`, `decimal_places`, and `units`.

Current issues:
1. `SVT_THR_STEP` default 0.0025 produces 1% per detent — too coarse for accurate ESC testing.
2. `decimal_places = 0` truncates sub-1% values to 0 on the display.
3. `start_running()` resets `_output_throttle = 0.0f`; RUNNING display uses `_output_throttle` during STARTING ramp → display drops to 0% on button press.
4. STOPPING ramp runs `_output_throttle` to 0, then state returns to CONFIG which displays `_target_throttle` (unchanged, e.g. 50%) → display jumps up when motors stop.

## Goals / Non-Goals

**Goals:**
- 0.25% per detent resolution, visible on the two-decimal display
- Seamless display value across CONFIG↔RUNNING transitions
- Throttle resets to 0 after any stop (safer default for re-arm)

**Non-Goals:**
- Changing the ramp filter or ramp tau
- Per-channel throttle control
- Adding a "last throttle" memory/restore feature

## Decisions

### SVT_THR_STEP default: 0.000625
0.000625 × 4 pulses/detent = 0.0025 = 0.25%. The param range (0.0001–0.02) is unchanged; users can increase it if 0.25% is too fine. Board default in `rc.board_defaults` updated to match.

### decimal_places = 2 for throttle readout
2 dp is the minimum to render 0.25% as "0.25" rather than "0.3" (1 dp) or "0" (0 dp). At 100% the field shows "100.00" which fits within the display's numeric area.

### Show _target_throttle during STARTING ramp
The commanded value (`_target_throttle`) is what the user set and what they expect to see. The ramp is a physical safety smoothing — it should not alter the displayed command. During STOPPING the actual output (`_output_throttle`) is shown so the user sees the ramp-down progress.

Conditional in `publish_display()`:
```cpp
d.numeric_value = ((_ramp_state == RampState::STARTING)
                   ? _target_throttle : _output_throttle) * 100.0f;
```

### Reset _target_throttle = 0 on stop completion
After a stop the system returns to CONFIG ready for the next test. Starting from 0 is safer than implicitly resuming at the last set-point. Both stop paths (immediate in `stop_running()`, ramp-down in `Run()`) set `_target_throttle = 0.0f`.

## Risks / Trade-offs

- [Resolution increase] → At 0.25%/detent, dialling from 0 to 100% takes 400 detents. Mitigation: SVT_THR_STEP is a param — users can increase it for coarse adjustment.
- [_target_throttle reset] → Users who want to restart at the same throttle must re-dial. Considered: not resetting; rejected because seeing 50% on a stopped system is confusing and potentially unsafe.
- [decimal_places = 2 field width] → "100.00 % SET" may be wider than the display numeric widget. The ST7789 driver is responsible for truncation if needed; no change required there.
