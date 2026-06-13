## Context

`rotary_encoder_event_s` carries separate boolean fields: `push_rising` (button pressed down) and `push_falling` (button released). The `servo_test` Run() loop drains all queued encoder events in a `while` loop each 20 ms tick. Currently, CONFIG triggers `start_running()` on `push_rising`.

The change is localised entirely to the CONFIG encoder block in `Run()`.

## Goals / Non-Goals

**Goals:**
- Start only fires when the operator completes a full press-and-release cycle
- K0 press during a held main button cancels the pending start
- Stop (RUNNING → CONFIG graceful stop) is unaffected

**Non-Goals:**
- Visual indicator that the button is "arming" (no progress bar or display change while button held)
- Debounce logic (the EC11 driver handles that)
- Changing SETUP entry or e-stop behaviour

## Decisions

### Single `_push_pending` bool, no timer
A simple edge-pair latch is sufficient. The operator presses down → `_push_pending = true`; operator releases → start fires. No timeout needed because: (a) the operator can hold indefinitely without risk, and (b) adding a timeout would require an additional hrt timestamp and complicates state cleanup on exit. Benefit: zero CPU overhead, trivially correct.

### Reset `_push_pending` in `enter_setup()` not in `exit_setup()`
`enter_setup()` is the point where CONFIG is left. Clearing on entry to SETUP ensures no stale pending survives the round-trip CONFIG→SETUP→CONFIG. `exit_setup()` does not need a clear because `_push_pending` is already false (it was cleared on `enter_setup()`).

### Stop remains on `push_rising`
Stopping requires fast response — the operator may want to cut motors quickly. Requiring a release for stop would add latency and could be dangerous. Only the start path gets the release requirement.

### `push_falling` in SETUP / RUNNING ignored
The existing code already ignores `push_falling` in RUNNING and SETUP states. No change needed there — the `_push_pending` flag is only checked in the CONFIG branch.

## Risks / Trade-offs

- [Latency on start] → Start now fires ~10–100 ms later (button release vs press). Acceptable for a bench tester where the operator is deliberate.
- [push_falling arrives in wrong state] → If the button is held while transitioning to SETUP via K0, `enter_setup()` clears `_push_pending`. When the button is released in SETUP state, `push_falling` hits the SETUP branch which doesn't check `_push_pending` — no action taken. Correct behaviour.
- [Stale event across 20 ms tick boundary] → The `while` loop drains all queued events each tick. If `push_rising` and `push_falling` arrive in the same tick (very fast press), both are processed in order within the same loop iteration and `start_running()` fires correctly.
