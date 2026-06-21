# Active Flight Path Safety Fixes

> Date: 2026-06-09, branch `rtos-refactor`.
> Scope: the **currently active** motor-output path only (stick → angle P → rate PID →
> quad-X mix → DShot). No EKF, no PositionController / VelocityController / RTH /
> CascadedController wiring, no autonomy, no receiver-map, motor-order, or
> motor-direction changes, no failsafe removal, no gain changes.
> Build verified: `pio run -e esp32-s3-mini` → SUCCESS (no new warnings).
> The `-pidweb` env could not be built on this machine only because the gitignored
> `src/pidweb_secrets.h` is absent (pre-existing, by design); it shares every changed
> file with the env that builds clean.

---

## 1. Original safety problems

| # | Problem | Where it lived |
|---|---|---|
| P1 | **Zero throttle while ARMED called `forceMotorStop()`** — props stopped dead and all PID state was wiped, even airborne. A mid-air throttle chop meant total thrust loss, controller amnesia, then a slew-limited respool from zero. | `updateControlLoop()`, old `if (smoothedThrottle == 0 \|\| !imuValidSnap) { forceMotorStop(); return; }` |
| P2 | **No mixer desaturation** — each motor was clamped independently to [48..2047] (or 0). When one motor clipped, the commanded roll/pitch/yaw moments silently distorted, exactly at high throttle + disturbance where attitude authority matters most. No saturation visibility anywhere. | `updateControlLoop()` quad-X mix + `clampMotorRaw()` |
| P3 | **Integrators unprotected against motor-level saturation and ground windup** — the PID class had output-level conditional integration, but nothing stopped I from accumulating while motors were clipped at a rail or while sitting on the ground at low throttle. No NaN/Inf input guard in the PID class. | `FcuPidController::update()`, `updateControlLoop()` |
| P4 | **PID resets were not transition events** — `resetPidOutputs()` ran on *every* zero-throttle tick (via `forceMotorStop()`), so trim was discarded continuously rather than at disarm/failsafe/arming boundaries. No record of why/when a reset happened. | `forceMotorStop()` call sites |
| P5 | **No single place to see the flight-path state** — armed/idle/failsafe/saturation/reset-reason required reading four different gated debug streams. | logging |
| — | **Failsafe "armed" gap created by the P1 fix itself** (found during this work): `FailsafeManager` treats `armed = appliedThrottlePercent > 0`. With armed-idle the props spin while that reads 0, which would have skipped the unsafe-tilt / IMU-invalid / loop-overrun failsafes at idle. Fixed as part of P1. | `applyFailsafeIfNeeded()` |

## 2. Files and functions changed

| File | What changed |
|---|---|
| [include/fcu_pid.h](include/fcu_pid.h) | `FcuPidController::update()` gained `allowIntegration` (default `true` — `AltitudeController` and every other caller unchanged) and a non-finite input guard. Conditional-integration commit now also requires `allowIntegration`. |
| [include/flight_control.h](include/flight_control.h) | `PidRuntime` gained: 6 saturation flags + `mixerSatPrevTick` (the one-tick-delayed I-gate input), `armedIdleActive`, `lastResetReason`/`lastResetMs`/`resetCount`, `prevAllowFlight`. All documented in-struct. |
| [include/motor_module.h](include/motor_module.h) | `forceMotorStop(const char* reason = "unspecified")` + a contract comment: transition-only, never a low-throttle response. |
| [include/control_protocol.h](include/control_protocol.h) | `TelemetryAuxPacket.reserved[1]` → `mixerSatFlags` (same offset/size — old remotes simply ignore it) + 7 `kTelemetrySat*` bit constants. |
| [src/main.cpp](src/main.cpp) | All behavior changes — itemized below. |
| [platformio.ini](platformio.ini) | Doc block for `FCU_ARMED_IDLE_ENABLE` (default ON in code; commented `=0` line restores legacy behavior). |

`main.cpp` changes by function:
- **Constants block**: `FCU_ARMED_IDLE_ENABLE` / `ARMED_IDLE_ENABLED`, `ARMED_IDLE_MOTOR_RAW` (= existing `MOTOR_OUTPUT_MIN_ACTIVE_RAW` = DShot 48 — no new magic value), `PID_ITERM_MIN_THROTTLE_PCT` (= 5, matching the existing 5% "meaningful throttle" manual-override threshold).
- **`resetPidOutputs(reason)`**: records reason/time/count on reason *change*; policy comment (transition-only). Callers hold `gFlightMux`; no Serial inside.
- **`forceMotorStop(reason)`**: passes reason through. Every call site tagged: `"esc_settle"`, `"imu_invalid"`, `"disarmed"`, `"zero_throttle_legacy"`, `"autonomy_stop"`, `"touchdown_disarm"`, `"motor_test_complete"`, `"motor_test_abort"`, `"failsafe_motors_idle"`, `"failsafe_release_complete"`, `"fatal_task_create"`. Other reset sites: `"gyro_cal_complete"`, `"pidweb_gain_apply"`, `"failsafe_soft_release"`, `"failsafe_clear"`, `"arming"`.
- **`updateControlLoop()`**: arming-edge PID reset; IMU-invalid stop split from zero-throttle handling; armed-idle branch; integrator gate; per-axis PID-clamp flags; mixer desaturation; saturation flag publishing; airborne heuristic; `[FLT_STATE]` 2 Hz log; `[TUNE_DBG]` extended.
- **`applyFailsafeIfNeeded()`**: `fsIn.armed` now includes `armedIdleActive`.
- **`mixerSatFlagsByte()`** (new, tiny): packs the flags for telemetry/debug.
- **`sendTelemetry()`**: fills `aux.mixerSatFlags`.

## 3. The exact fixes

### P1 — armed zero-throttle motor stop → armed idle

Old (`updateControlLoop`):
```cpp
if (smoothedThrottle == 0 || !imuValidSnap) {
  forceMotorStop();
  return;
}
```

New:
```cpp
if (!imuValidSnap) {
  forceMotorStop("imu_invalid");      // never fly blind — unchanged hard stop
  return;
}
bool armedIdleActive = false;
if (smoothedThrottle == 0) {
  if (!allowFlight || !ARMED_IDLE_ENABLED) {
    forceMotorStop(!allowFlight ? "disarmed" : "zero_throttle_legacy");
    return;                            // DISARMED: the legitimate hard stop
  }
  armedIdleActive = true;              // ARMED: hold idle, PIDs stay live
}
```
plus, in the mixer: `if (armedIdleActive) base = ARMED_IDLE_MOTOR_RAW;` and an
armed floor of `MOTOR_OUTPUT_MIN_ACTIVE_RAW` on every motor (the mixer is only
reachable while armed — every disarm/failsafe path returns earlier).

Hard motor stops now happen **only** on: disarm (switch off / not flight mode /
safe-boot incomplete), failsafe paths (soft-release ramp, IMU invalid), confirmed
touchdown (`LandingController.requestDisarm`), explicit autonomy STOP, ESC startup
settle, pid-web motor-test end, fatal task-create. Each is reason-tagged.

PID resets now happen **only** on: disarm/each of the stops above, failsafe entry
and clear, **arming edge** (new — one clean reset when `allowFlight` goes
false→true), gyro-cal completion, web-tuner gain apply. Rate PIDs are deliberately
**not** reset on flight-mode transitions (ATK engage/abort): the inner loop must
fly through the transition; a reset there would inject a motor transient mid-air.
The outer altitude controller (`gAltCtrl`) is what resets at mode boundaries
(pre-existing behavior, kept).

### P2 — mixer saturation/desaturation

Old: four independent `clampMotorRaw(base ± corrections)` — clipping distorted moments.

New three-stage handling (correction composition, motor order, directions, and
the ×1.3 front-pitch bias are **unchanged**):
1. **Spread scaling**: if `max(corr)−min(corr)` exceeds the whole motor range,
   all corrections scale uniformly (moment *ratios* preserved; flagged
   `corrScaled`; nearly unreachable with the ±240 PID clamps — worst case ≈1700
   < range ≈2000).
2. **Upper-bound base shift**: if the highest motor would exceed DShot 2047,
   `base` is reduced so it doesn't — full attitude moments are preserved at the
   cost of collective thrust (flagged `maxSat`). This is the standard
   Betaflight/ArduPilot trade: altitude is recoverable, attitude is not.
3. **Lower bound**: per-motor clamp at the armed floor (48), flagged `minSat`.
   Deliberately **not** compensated by raising base — that would be air-mode
   collective boost, too aggressive to introduce untested (TODO below).

Saturation flags (`minSat`/`maxSat`/`corrScaled` + per-axis `rollPidSat`/
`pitchPidSat`/`yawPidSat` from the PID output clamps) are published every tick to
`gState.pid`, the `[FLT_STATE]`/`[TUNE_DBG]` logs, and telemetry
(`TelemetryAuxPacket.mixerSatFlags`).

### P3 — PID safety

- **dt units verified, no change needed**: `dtSeconds = elapsedUs × 1e-6`,
  already clamped to [1000, 10000] µs by `CONTROL_LOOP_DT_MIN_US/MAX_US` before
  the PIDs run. Units audit: gyro dps ↔ rate setpoints dps ✓; attitude degrees ↔
  angle setpoints degrees ✓; PID outputs in raw DShot counts ✓.
- **Non-finite guard** (new, in `FcuPidController::update`): NaN/Inf setpoint,
  measurement, or dt → hold the previous output for the tick and skip all state
  updates (a 2 ms stale output is safer than snapping a motor to zero mid-air).
- **Integrator gating** (new): I is **frozen — held, never reset —** while
  `smoothedThrottle < 5 %` (ground / armed idle: integrating attitude error with
  no authority is pure windup) or while the mixer reported any saturation on the
  previous tick (a clipped motor cannot realize more moment). P and D stay fully
  active; the PID-internal conditional integration still applies on top. The
  feedback is one tick (2 ms) delayed by construction.
- **Disarmed / failsafe windup**: while disarmed the loop returns before the PIDs
  run and state is reset; non-link failsafes return through the soft-release
  branch before the PIDs; the controlled link-loss descent intentionally keeps
  PIDs (and I) active because it is actively stabilizing the descent.

### P5 — visibility

- **`[FLT_STATE]`** (new, always on, 2 Hz, ~300 B/s, backpressure-gated): armed,
  armed-idle, airborne-heuristic, failsafe+reason, packet throttle + applied
  (smoothed) throttle, loop dt µs, all six saturation flags, four motor outputs,
  last PID reset reason@time and count. Emitted from `flightTask` after every
  control tick so it prints in **every** state — disarmed, failsafe, ESC settle —
  not just while flying (relocated during the second-pass audit; see
  ACTIVE_FLIGHT_PATH_REGRESSION_REVIEW.md).
- **`[TUNE_DBG]`** (existing, opt-in) extended with: armed, idle, dt µs, packed
  saturation byte, and the four **pre-clamp** mixer values next to the existing
  post-clamp motor outputs.
- **Telemetry**: `TelemetryAuxPacket.mixerSatFlags` (was the reserved byte) now
  carries min/max/scaled/axis-saturation + armed-idle in flight, on the existing
  nRF24 link.
- **Airborne heuristic** `gAirborneLikely` (new): latched after ≥500 ms at ≥20 %
  commanded throttle while armed; cleared only on disarm. **Logging/telemetry
  only — explicitly not a control input** (TODO: validate against ToF/EKF ground
  truth before any control use).

## 4. What was intentionally NOT changed

- **No autonomy/EKF/navigation wiring**: `VelocityController`, `PositionController`,
  `ReturnToHome`, `CascadedController` remain un-called; the EKF remains shadow-only.
- **Motor order, directions, pins, mixer signs, front-pitch bias**: untouched.
- **Receiver channel mapping**: untouched (CRSF CH5-arm bring-up map kept).
- **All failsafe logic kept**; the only failsafe edit *adds* coverage (armed-idle
  counts as armed for tilt/IMU/loop-overrun checks).
- **No gain changes**; kd stays 0; angle/rate limits unchanged. Nothing got more
  aggressive — at any nonzero throttle with no clipping, the mixer output is
  numerically identical to before.
- **Air-mode-style lower-bound boost**: deliberately not implemented (TODO).
- **Rate-PID reset on mode change**: deliberately not added (see P1 rationale).
- **`gAltCtrl.reset()` every tick while alt-hold is inactive**: left as is — it
  resets an *inactive* controller (harmless), unlike the armed in-flight resets
  this patch removes.
- **Observer FSM's `armed` definition**: left as `appliedThrottlePercent > 0`
  (observer-only, no motor authority; changing it would only relabel logs).
- **Pre-existing oddity, documented not fixed (autonomy path, out of scope):**
  the Pi `LAND` command's `effectiveThrottle − 5` ramp runs *after* the slew
  filter has already consumed `effectiveThrottle`, so the reduction only reaches
  `appliedThrottlePercent` telemetry — never the mixer — and is recomputed from
  scratch each tick. The autonomy UART is disabled in the current build
  (`FCU_ENABLE_AUTONOMY_UART=0`). TODO when autonomy is wired: feed the land ramp
  into the throttle target *before* the slew filter, or hand `LAND` to
  `LandingController`.

## 5. Remaining risks / TODOs

1. **TODO(hardware): DShot-48 idle spin reliability.** Some ESC/motor combos need
   55–80 to start/hold rotation under prop load. Verify on the bench (props off,
   then props on + restrained) and raise `ARMED_IDLE_MOTOR_RAW` if needed.
2. **Behavior change — arming spins props.** CH5-arm after safe-boot now starts
   all four props at idle on the ground (market-standard, but new *here*). The
   pilot must expect it; `FCU_ARMED_IDLE_ENABLE=0` restores legacy behavior.
3. **Armed idle ground twitch.** At idle with the craft tilted, the angle loop
   commands a correction; upward-correcting motors will hum above idle (floor
   clips the rest, `minSat` + frozen I prevent windup). Same behavior as
   Betaflight-without-airmode; verify it looks sane props-off.
4. **I-freeze during sustained floor-riding.** In a low-throttle descent with a
   motor pinned at the floor, I stops adapting trim for those ticks (windup
   protection wins by design). If pitch trim visibly degrades in low-throttle
   descents, revisit with per-axis directional gating.
5. **TODO(airmode):** lower-bound base shift (collective boost to preserve
   authority at idle) — only after armed-idle + desat are flight-proven.
6. **Zero-throttle descent has minimal attitude authority** (corrections can only
   raise motors off the floor). Armed idle makes a chop *recoverable*, not
   *stabilized* — that requires the air-mode TODO.
7. **`gAirborneLikely` is a heuristic** — logging only until validated.
8. **One-tick (2 ms) delay on the saturation→I-gate feedback** — inherent to the
   design; acceptable at 500 Hz.
9. **Remote telemetry decoder** should learn the new `mixerSatFlags` byte (old
   decoders ignore it safely).

## 6. Props-off test plan (run before ANY motor test)

Flash `esp32-s3-mini`, USB serial attached, **props off**, battery on a current-limited
supply if available. Watch `[FLT_STATE]` throughout.

1. **Boot/settle**: confirm 3 s ESC settle (`rst=esc_settle`), then `arm=0 idle=0`,
   motors silent, `rst=disarmed` once the loop runs.
2. **Safe-boot + arm**: throttle low, CH5 high → expect `[FLT_STATE] arm=1 idle=1`,
   all four motors spinning at raw 48 (`m=[48,48,48,48]`), `rst=arming@…` exactly
   once, reset count stable thereafter. **This is the new behavior — verify every
   motor actually rotates at 48** (TODO #1).
3. **Disarm**: CH5 low → motors stop immediately, `arm=0 idle=0`, `rst=disarmed`.
4. **Throttle chop while armed**: raise to ~30 %, hold, snap to 0 → motors drop to
   idle (NOT stop), `idle=1`, PID terms in `[TUNE]` show I frozen (flat) but P/D
   still responding to hand-rotation; raise throttle → instant resume, no reset
   logged.
5. **Idle twitch check**: armed at idle, tilt the frame by hand → upward-side
   motors rise above 48, others stay at floor, `sat=[mn1 …]`, I terms stay frozen
   (`[TUNE]` I column flat at low throttle).
6. **Integrator freeze vs resume**: armed, throttle 10 %, hold the frame tilted →
   I should accumulate (above 5 % gate, no saturation); drop throttle to 3 % → I
   freezes at its value (not zeroed); back to 10 % → resumes from the same value.
7. **Upper-bound desat**: throttle to 100 % and twist the frame hard → expect
   `mx1` flags in `[FLT_STATE]`, motor spread preserved (high motor pinned at
   2047, others tracking below it), no motor commanded past 2047.
8. **Failsafe at idle (new coverage)**: armed at idle, tilt past 60° → unsafe-tilt
   failsafe must latch (`fs=1 r=5`), motors soft-release to stop; clear via the
   failsafe-clear path only at zero throttle.
9. **Link-kill drill**: armed at ~30 %, power off the TX → 350 ms latch, hold then
   ramp/descent path as before, `rst=failsafe_soft_release`; confirm armed idle
   does NOT re-engage until link returns and you re-arm.
10. **IMU-invalid drill**: (bench jig) disconnect IMU SPI while armed at idle →
    8-tick grace then hard stop, `rst=imu_invalid`, failsafe reason 8.
11. **Legacy-mode regression**: rebuild once with `-D FCU_ARMED_IDLE_ENABLE=0` and
    confirm step 2 yields silent motors at zero throttle (old behavior intact),
    then return to the default build.

## 7. ESC/motor test plan (no props)

1. `fcu_bench_test` env: motor order + direction verification (unchanged code
   path — expected identical results; this confirms the mixer refactor preserved
   composition).
2. Main env, armed idle: measure per-motor RPM (audible/optical) at raw 48 for
   ≥60 s — look for stalls, cogging, ESC desyncs/restarts, or hot motors. If any
   motor stalls at 48, raise `ARMED_IDLE_MOTOR_RAW` (e.g. 60) and re-run.
3. pid-web motor-test (bench env, when secrets present): single-motor spin still
   starts/stops cleanly; `rst=motor_test_complete` / `motor_test_abort` logged.
4. Throttle staircase 0→10→25→50→75→100→0 %: confirm slew behavior unchanged,
   no `mx`/`sc` flags at steady state, motors return to *idle* (not stop) at the
   final 0 % while armed, and stop only at disarm.
5. (When BDShot pull-ups are fitted, separate session) re-run with
   `FCU_DSHOT_BIDIR=1` to confirm idle-spin coexists with eRPM telemetry.

## 8. Tether-only test plan (after props-off passes; props on, craft restrained)

Short leash/tether over grass or foam, current-limited line of retreat, TX failsafe
pre-checked, `FCU_TUNING_LOG_HZ=100` active.

1. **Arm with props**: confirm idle spin is steady and the craft stays planted
   (idle thrust must be << weight — visually confirm, this is the no-airmode
   guarantee).
2. **Hover at ~35 %**: verify behavior is indistinguishable from the pre-patch
   build (no clipping → identical mixer math). Watch `sat=[…]` — should be all
   zeros in calm hover.
3. **Throttle chop at ~1 m, tethered**: chop to zero for <1 s, then recover —
   props must keep spinning, recovery must be immediate with no PID-reset bump
   (`rst` unchanged, reset count stable).
4. **Disturbance + high throttle**: brief 70–100 % pulls with stick deflection —
   expect occasional `mx1` flags, no attitude let-go, no oscillation onset.
5. **Failsafe end-to-end with props**: TX off at low tethered hover → confirm the
   safe-landing/ramp path and that motors stop only via the failsafe transition.
6. Only after all of the above: untethered low hover over soft ground.

## 9. Logs to collect before any real flight

1. Full boot banner through first arm/disarm cycle (`[FLT_STATE]`, `rst=` lines).
2. ≥60 s of armed idle on the ground: `[FLT_STATE]` + `[TUNE]` (verify I frozen,
   motors at 48, no failsafe).
3. The throttle-chop bench capture (step 6.4) showing idle hold + clean resume.
4. A `[TUNE_DBG]` session (bench, `FCU_TUNING_DEBUG=1`) covering idle, mid, and
   full throttle with hand disturbances — keep the `pre=[…]` vs `m=[…]` pairs to
   verify desat math on real numbers.
5. The 100 % throttle + disturbance capture showing `mx1` flags with preserved
   motor spread (desat working).
6. The failsafe drill captures (link kill, tilt latch at idle, IMU drill) with
   reasons and `rst=` transitions.
7. Tethered hover `[TUNE]` CSV — to confirm hover behavior is unchanged vs the
   pre-patch baseline before trusting the new build in free flight.
8. Telemetry capture on the remote showing `mixerSatFlags` arriving (byte 31 of
   the aux packet).
