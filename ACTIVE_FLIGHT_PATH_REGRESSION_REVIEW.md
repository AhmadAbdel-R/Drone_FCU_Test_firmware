# Active Flight Path — Second-Pass Regression Review

> Date: 2026-06-09, branch `rtos-refactor`.
> Scope: audit of the safety patches documented in
> [ACTIVE_FLIGHT_PATH_FIXES.md](ACTIVE_FLIGHT_PATH_FIXES.md) — armed idle,
> stop/reset reasons, integrator gating, mixer desaturation, saturation flags,
> failsafe evaluation, logging/telemetry. No new features; no EKF / RTH /
> Position / Velocity / Cascaded wiring; no gain, receiver-map, motor-order, or
> motor-direction changes.

## 1. Bugs found in the patch (all fixed in this pass)

### BUG-1 — `[FLT_STATE]` went silent in exactly the states it exists for
The 2 Hz state line was emitted at the tail of `updateControlLoop()`, *after*
`applyMotorOutputs()` — a point only reached on armed mixer ticks. Every early
return (disarmed, failsafe soft-release, IMU invalid, ESC settle) skipped it, so
the always-on log printed nothing while disarmed or failsafed — the exact
moments you need to read armed state, failsafe reason, and the PID reset reason.
**Fix:** emission moved to a new `emitFlightStateLine()` called from
`flightTask` after every control tick, reading only published `gState` (motor
snapshot under `gFlightMux`, control fields under `gControlMux`, flight-task-
written byte flags plain). A new `PidRuntime::lastDtUs` mirrors the clamped loop
dt so the line can report timing without `updateControlLoop` locals. Same 2 Hz
rate limit + `serialHasRoom` backpressure gate. (Trade: the line now shows
packet + applied throttle rather than the in-tick `effectiveThrottle`, and
during a pid-web single-motor test the `continue` path skips it — `[MOTOR_TEST]`
lines cover that window.)

### BUG-2 — Stale saturation flags / `mixerSatPrevTick` / `armedIdleActive` across stop gaps
The six saturation flags, the integrator-gate input `mixerSatPrevTick`, and
`armedIdleActive` are only refreshed by mixer-reaching ticks. Any stop
(disarm, failsafe, IMU invalid) froze their last values: telemetry showed
last-flight saturation while sitting disarmed, and a saturation flag latched on
the final airborne tick would gate the integrators on the **first tick after the
next arming** (stale-state leak across flights — also the answer to audit
check #5: without this fix, one saturation event *could* outlive its tick
across a stop gap; it still could never persist through a flight, since every
mixer tick rewrites it).
**Fix:** `resetPidOutputs()` now clears all six flags, `mixerSatPrevTick`, and
`armedIdleActive`. Every stop/transition path funnels through it, so re-arming
always starts with an ungated integrator and clean telemetry.

### BUG-3 — Bench-tuner gates accepted while props spin at armed idle
`pidWebWriteSafe()`, `requestGyroCalibration()`, and `requestMotorSpin()` all
used `appliedThrottlePercent == 0` (plus observer-FSM `IDLE`, whose `armed`
input is *also* `appliedThrottlePercent > 0`) as proof that motors are quiet.
Armed idle broke that invariant: the props turn at DShot 48 while applied
throttle reads 0 and the FSM stays `IDLE`. On the bench (`-pidweb`) image this
would have allowed live gain apply + PID reset, a gyro-bias calibration
corrupted by idle vibration, or a single-motor test command — all beside four
spinning props.
**Fix:** all three gates now additionally require
`!anyMotorOutputActive(gState.motorRaw)` (snapshot under `gFlightMux`). The
flight image never exposes these paths; the fix protects the bench image.

### BUG-4 — `gAirborneLikely` latched forever after disarm
The heuristic's updater also lived in the armed-only mixer path; its only
clearing branch (`!allowFlight`) was unreachable once the disarm path returned
earlier in the function, so `air=1` would persist on the (now always-printing)
state line after landing + disarm.
**Fix:** `forceMotorStop()` clears `gAirborneLikely` — every final stop
(disarm, failsafe completion, touchdown) drops the latch. A mid-air failsafe
stop clears it too; that is acceptable for a logging-only flag and it re-latches
on recovery.

## 2. Audit results for the ten requested checks

1. **Motors stay spinning when they should hard-stop — no case found.**
   Disarm: `allowFlight` false ⇒ `effectiveThrottle` 0 ⇒ the slew filter's
   explicit zero-target branch snaps `smoothedThrottle` to 0 the same tick ⇒
   `forceMotorStop("disarmed")`. (Verified: `smoothedThrottle > 0` implies
   `allowFlight`, since every nonzero `effectiveThrottle` assignment is inside
   an `allowFlight` branch and zero target bypasses the slew.) Non-link
   failsafe: early return into the soft-release ramp before any mixing.
   Link-loss ramp reaching zero: caught by the pre-existing
   `effectiveThrottle == 0` soft-release branch *before* the armed-idle check.
   ESC settle, IMU invalid, touchdown, autonomy STOP: unchanged hard stops.
   Armed idle itself requires the full `allowFlight` conjunction.
2. **Motors hard-stop while armed + airborne — only by design.** Remaining
   in-air stops: IMU invalid (failsafe; 8-tick grace), pilot switch-disarm
   (pre-existing, standard), confirmed touchdown, explicit Pi STOP (compiled
   out). The throttle-chop stop — the original defect — is gone; verified the
   armed zero-throttle path now mixes idle with live PIDs.
3. **Continuous PID resets in normal flight — none.** No reset call site is
   reachable on the armed path except transition events. While disarmed,
   `forceMotorStop` repeats at 500 Hz but the reset is idempotent and the
   reason bookkeeping only updates on change.
4. **Resets on disarm/failsafe/mode change — covered.** Disarm
   (`"disarmed"`), failsafe entry (`"failsafe_soft_release"`), failsafe clear,
   arming edge (`"arming"`), gyro-cal, gain apply. Rate PIDs deliberately do
   NOT reset on ATK/alt-hold mode transitions (continuity through an in-air
   handoff; a reset there would inject a motor transient) — the outer
   `gAltCtrl` is what resets at mode boundaries (pre-existing, kept).
5. **Integrator frozen forever — impossible after BUG-2 fix.** Gate inputs are
   `smoothedThrottle` (recomputed every tick) and `mixerSatPrevTick`
   (rewritten by every mixer tick, cleared by every reset). Worst case is one
   2 ms tick of stale gating.
6. **Stale/wrong saturation flags — fixed (BUG-2) and verified.** Flags are
   recomputed from scratch each mixer tick (no `|=` accumulation), cleared on
   every reset, and the telemetry byte (`mixerSatFlagsByte`) reads live values.
7. **Desaturation math — correct, verified term-by-term.**
   `corr[]` equals the legacy per-motor composition exactly (M1
   `+roll −pitchFront +yaw`, M2 `+roll +pitchRear −yaw`, M3 `−roll −pitchFront
   −yaw`, M4 `−roll +pitchRear +yaw`); array order ⇒ `applyMotorOutputs`
   mapping ⇒ GPIO mapping unchanged; axis signs still come solely from
   `MIX_*_SIGN`. Uniform spread-scaling preserves moment ratios; the
   upper-bound shift subtracts the same value from all motors (moments
   preserved). Bound proof: not all four corrections can be positive
   (summing pairs forces `pitchFront < 0` and `pitch > 0`, a contradiction), so
   `corrMin ≤ 0`; after scaling `spread ≤ ceil − floor`, hence
   `floor + corrMax ≤ floor + spread ≤ ceil` — the floor clamp can never push
   the top motor past 2047. `clampMotorRaw` remains as belt-and-suspenders.
   Authority is unchanged: PID outputs still clamp at ±240 and at any
   unclipped operating point the outputs are numerically identical to the
   legacy mixer.
8. **Armed idle bypassing failsafe/arming — closed.** Idle requires the full
   arming conjunction (mode + switch + safe-boot + RPM-filter-ok + no
   uncontrolled failsafe); `FailsafeManager` counts idle as armed (tilt / IMU /
   loop-overrun checks active while props spin); ESC-settle precedes it; the
   bench gates are hardened (BUG-3). A latched failsafe at idle exits through
   the normal soft-release ramp.
9. **Logging cannot block the 500 Hz loop.** All new output: 2 Hz rate limit,
   `serialHasRoom` gate (skip + backpressure counter when the CDC buffer is
   short), no prints inside critical sections, no prints in `resetPidOutputs`
   (callers hold `gFlightMux`). `emitFlightStateLine` adds two short mutex
   snapshots at 2 Hz. `[TUNE_DBG]` additions are compile-gated off by default.
10. **`FCU_ARMED_IDLE_ENABLE=0` — compiles and restores legacy behavior.**
    Verified by building with the flag (see §4). `ARMED_IDLE_ENABLED` is a
    `constexpr bool` used in plain conditionals — no preprocessor-only paths to
    rot.

## 3. Files changed in this pass

| File | Change |
|---|---|
| [src/main.cpp](src/main.cpp) | BUG-1: `[FLT_STATE]` block removed from `updateControlLoop`, new `emitFlightStateLine()` + call in `flightTask`, `lastDtUs` mirror. BUG-2: flag clearing in `resetPidOutputs()`. BUG-3: `anyMotorOutputActive` forward declaration + checks in `pidWebWriteSafe` / `requestGyroCalibration` / `requestMotorSpin`. BUG-4: `gAirborneLikely` cleared in `forceMotorStop()`. |
| [include/flight_control.h](include/flight_control.h) | `PidRuntime::lastDtUs` (for BUG-1). |
| [ACTIVE_FLIGHT_PATH_FIXES.md](ACTIVE_FLIGHT_PATH_FIXES.md) | `[FLT_STATE]` description updated to the relocated form. |

## 4. Tests / builds run

| Check | Result |
|---|---|
| `pio run -e esp32-s3-mini` (default, all fixes) | **SUCCESS**, no new warnings |
| `pio run -e esp32-s3-mini` with `PLATFORMIO_BUILD_FLAGS="-D FCU_ARMED_IDLE_ENABLE=0"` | **SUCCESS** (legacy stop-at-zero restored, compiles clean) |
| Native EKF sim suite (`ziglang c++ … src/ekf_sim_test.cpp`; the `native_ekf_sim` env needs a host gcc this machine lacks) | **9/9 PASS** (S1–S9, incl. NaN-reject, GPS-glitch gate, closed-loop vel hold) — confirms no collateral damage to the sim stack |
| Call-graph grep: `gVelocityCtrl.update` / `gPositionCtrl.update` / `gRth.update` / `gRth.arm` / `gCascade.` | **Zero matches** — no autonomy controller is wired to the motor path |
| `gEkf.*` usage | Unchanged shadow set only (predict/measurement/getState/getHealth + init/origin) — nothing feeds the mixer |
| `-pidweb` env | Not buildable on this machine (gitignored `src/pidweb_secrets.h` absent — pre-existing, by design). It shares every changed file with the env that builds clean; the BUG-3 functions compile inside `ENABLE_PID_WEBSERVER` only in that env, so flash the pidweb image once where secrets exist before relying on it. |

## 5. Verified-correct items (no change needed)

- **Disarm latency:** zero-target slew bypass guarantees a one-tick (≤2 ms)
  path from switch-off to `forceMotorStop`, idle or not.
- **Arming-edge reset:** fires exactly once per `allowFlight` rising edge;
  placed after `allowFlight` computation and before all armed-path branches;
  `prevAllowFlight` is updated on every tick that reaches it, including ticks
  that then early-return into the failsafe path.
- **Integrator gate inputs** are tick-fresh; the PID-internal conditional
  integration (directional, output-level) still applies independently.
- **Non-finite PID guard:** returns the previous terms without touching state;
  with int16-derived sensor scaling NaN is essentially unreachable in the live
  path, and the persistent-NaN case is owned by the IMU-invalid failsafe.
  (Known cosmetic: after a skipped tick, the next D-on-measurement sample spans
  two ticks — kd=0 today and the 100 Hz D-LPF would absorb it.)
- **Failsafe `armed` input** (`appliedThrottle > 0 || armedIdleActive`) reads
  the previous tick's idle flag (applyFailsafeIfNeeded runs first) — one-tick
  lag, conservative direction (flags armed no later than motors spin).
- **Telemetry layout:** `mixerSatFlags` occupies the old reserved byte; packet
  size static_assert unchanged; old remotes ignore the byte.

## 6. Remaining risks / known limitations

1. **DShot-48 idle spin is still hardware-unverified** (TODO in code) — the
   single biggest open item before trusting armed idle with props on.
2. **Armed idle engaging during a controlled link-loss descent**: if the
   landing controller plus alt-bias momentarily commands 0%, the craft holds
   idle for those ticks instead of the old stop/restart chatter — an
   improvement, but it has not flown.
3. **Pre-existing, out of scope (documented, not fixed):**
   - Safe-landing descent runs the **altitude PI on top of the
     LandingController** (`altHoldActive = true` in the link-loss branch feeds
     the `FCU_ENABLE_ALTITUDE_HOLD` block targeting `atk.targetMeters`, which
     can be 0 m), so two controllers tug on the same throttle during failsafe
     descent. Predates this patch series; review before the first link-loss
     flight test.
   - Pi `LAND`'s `effectiveThrottle − 5` runs after the slew filter consumed
     the value (never reaches the mixer; autonomy UART disabled).
   - A single corrupted-but-CRC-valid packet with `mode=0` would still disarm
     (pre-existing link design; CRSF bridge hysteresis mitigates).
4. **I-freeze during sustained floor-riding** (low-throttle descents) trades
   trim adaptation for windup protection — watch pitch trim in descents.
5. **No airmode lower-bound shift** (explicit TODO) — zero-throttle descents
   keep props spinning but with limited attitude authority.
6. **`gAirborneLikely` clears on any motor stop**, including a mid-air
   failsafe stop — fine for logging, never use for control without the
   documented hardware validation.

## 7. Props-off validation checklist (final, supersedes §6 of the fixes doc)

Flash default `esp32-s3-mini`, USB serial attached, **props off**. `[FLT_STATE]`
prints every 500 ms in **all** states — verify that first.

1. **Boot**: during the 3 s ESC settle expect `arm=0 idle=0 m=[0,0,0,0]`
   `rst=esc_settle`; after settle, `rst` stays (no churn), reset count `n`
   stable.
2. **Disarmed baseline**: `[FLT_STATE]` flowing with `arm=0 idle=0 air=0
   sat=[mn0 mx0 sc0 r0 p0 y0]` (BUG-2 check: flags must be all-zero, not
   last-session values).
3. **Arm (CH5 high after throttle-low safe-boot)**: one `rst=arming@t n=+1`;
   `arm=1 idle=1 m=[48,48,48,48]`; **all four motors physically rotating at
   DShot 48** (open TODO #1 — if any stalls, raise `ARMED_IDLE_MOTOR_RAW` and
   re-test).
4. **Disarm**: within one line, `arm=0 idle=0 m=[0,0,0,0] rst=disarmed`;
   saturation flags all zero (BUG-2); `air=0` (BUG-4).
5. **Throttle chop while armed**: 30 % → hold → snap to 0: motors drop to
   `m=[48,48,48,48]`, `idle=1`, **no `rst` change, `n` unchanged** (the
   original P1 defect — must NOT stop or reset); raise throttle → immediate
   resume.
6. **Idle tilt**: armed at idle, tilt the frame ⇒ upper-side motors rise,
   lower side pinned at 48, `sat=[mn1 …]`, `[TUNE]` I columns flat (low-throttle
   freeze); level the frame ⇒ `mn` returns to 0 next ticks (flags are
   per-tick, BUG-2/check 6).
7. **I freeze/hold/resume**: armed, 10 % throttle, hold tilted → I accumulates;
   drop to 3 % → I holds its value (not zeroed); back to 10 % → resumes from
   the held value.
8. **Upper desat**: 100 % throttle + hard hand-twist ⇒ `mx1`, top motor pinned
   ≤2047 with spread preserved (compare `[TUNE_DBG] pre=[]` vs `m=[]` if
   enabled).
9. **Failsafe at idle (new coverage)**: armed-idle, tilt past 60° ⇒ failsafe
   latches (`fs=1 r=5`), soft-release ramp to stop, `rst=failsafe_soft_release`
   then `failsafe_release_complete`; clear only accepted at throttle zero with
   motors stopped.
10. **Link-kill at 30 %**: TX off ⇒ 350 ms latch ⇒ hold/ramp or sensor descent;
    confirm motors never jump and end stopped; `[FLT_STATE]` keeps printing
    throughout (BUG-1).
11. **IMU drill**: disconnect IMU SPI while armed-idle ⇒ ≤16 ms to
    `rst=imu_invalid`, motors stop, `fs r=8`.
12. **Legacy regression build**: `PLATFORMIO_BUILD_FLAGS="-D FCU_ARMED_IDLE_ENABLE=0"`
    → step 3 must yield silent motors at zero throttle (`idle=0 m=[0,0,0,0]`),
    `rst=zero_throttle_legacy` while armed at zero throttle; then return to the
    default build.
13. **Bench image only** (`-pidweb`, where secrets exist): with the TX armed at
    idle, the web tuner must now REFUSE gain apply / gyro-cal / motor-spin
    (BUG-3); disarm ⇒ requests accepted again.

Only after 1–13 pass: proceed to the no-props ESC plan and the tether plan in
[ACTIVE_FLIGHT_PATH_FIXES.md](ACTIVE_FLIGHT_PATH_FIXES.md) §7–8.
