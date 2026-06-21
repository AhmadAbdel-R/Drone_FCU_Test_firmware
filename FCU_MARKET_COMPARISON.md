# FCU Market Comparison — ESP32-S3 F450 Firmware vs. Existing Flight Controllers

> Analysis date: 2026-06-09, branch `rtos-refactor`.
> Every claim below was verified against the actual source (file/line refs given).
> "Not wired" means the module compiles and is configured, but its update/step
> function is never called from the flight path — confirmed by grep, not assumption.

---

## 1. Summary — what this firmware is closest to

**Verdict: a custom hybrid.** No single market firmware matches it, but the parts map cleanly:

| Layer | Closest match | Evidence |
|---|---|---|
| **What actually flies today** (angle-P → rate-PID @ 500 Hz, Euler complementary filter, additive quad-X mix, ToF alt-hold) | **Cleanflight-era ANGLE mode / ArduCopter "Stabilize"**, with Betaflight-inspired filtering bolted on | `main.cpp:7070–7181`, `fcu_pid.h`, `main.cpp:3048` |
| **Inner-loop feature choices** (DShot300/RMT, dynamic notch, planned RPM filter, D-term LPF, D-on-measurement, ELRS/CRSF) | **Betaflight** | `platformio.ini:126–185`, `DynamicNotchFilter.h`, `RpmNotchFilter.h`, `crsf_receiver.h` |
| **Autonomy architecture** (mode taxonomy MANUAL/ALT_HOLD/POS_HOLD/RTH/LAND, RTL state machine, 15-state error-state EKF, 4-tier cascaded controller, failsafe ladder, companion-computer link) | **ArduPilot/PX4** — mostly *scaffolded, not active* | `flight_modes.h`, `return_to_home.h`, `ekf_estimator.h`, `cascaded_controller.h`, `failsafe_manager.h`, `autonomy_uart.h` |
| **Overall product intent** (always-leveled, 15° tilt cap, auto-takeoff, link-loss auto-land, camera gimbal on the FC, GPS hold/RTH roadmap) | **INAV / DJI-Naza operator model** | `auto_takeoff.h`, `camera_gimbal.h`, `main.cpp:6914–6936` |

**One-line answer:** the *flying core* is a simplified Cleanflight/early-Betaflight ANGLE-only
stack; the *architecture being built around it* is ArduPilot-shaped; the *closest overall
positioning* is INAV (racing-firmware lineage + onboard GPS autonomy).

**Maturity statement (honest):** this is bring-up/bench-grade firmware, not yet comparable
in maturity to any shipping FCU. Concretely: rate-loop D is held at zero, the velocity /
position / RTH / cascaded controllers and the EKF are compiled in but **disconnected from
the motors**, the flight-mode switch is a TODO stub on the active (CRSF) receiver
(`main.cpp:5979`), GPS velocity is not parsed, and the unified state machine is
observer-only. What *is* implemented is implemented carefully (failsafe latching,
anti-windup, sanitized gains, watchdogs, sim-tested EKF), which is above typical hobby
bring-up rigor — but the autonomous feature set exists mostly as unwired modules.

---

## 2. Verified system description (basis for all scores)

### Control structure
- **Attitude:** always angle (self-level) mode. Stick → angle setpoint clamped ±15°
  (`MAX_ANGLE_SETPOINT_DEG`, `main.cpp:1208`) → P-only outer angle loop (no outer I/D)
  → rate setpoint clamped ±240 °/s (`FCU_MAX_ANGLE_RATE_SETPOINT_DPS`) → inner rate PID
  on gyro (`main.cpp:7080–7088`). **There is no acro/rate-only mode** — the loop has no
  rate-mode branch.
- **Rate PID** (`fcu_pid.h`): P+I+D with conditional-integration anti-windup, integral
  clamp decoupled from output clamp (±144 vs ±240 raw), D-on-measurement enabled,
  100 Hz first-order D LPF. **kd = 0 in current tune** (deliberately, pending prop balance).
  Default gains are 0 — real gains come from NVS (FCU is the source of truth).
- **Yaw:** stick → yaw **rate** setpoint, ±100 °/s (`main.cpp:1235, 7072`). No heading hold.
- **Altitude hold:** PI on filtered VL53L1X ToF (kp=0.060, ki=0.025, kd=0), output is a
  ±30 % throttle bias added to a fixed 35 % hover base (`altitude_controller.h`,
  `auto_takeoff.h:50`). Engaged only through the auto-takeoff state machine; the throttle
  stick does **not** command climb rate in this mode (it aborts to manual instead).
- **Auto-takeoff:** full state machine — PRECHECK → ARMING → RAMP_UP (35 %/s) → ASCEND →
  ALT_HOLD → COMPLETE, with liftoff detection (≥0.15 m rise within 3 s else abort),
  15° takeoff tilt gate, ToF+baro cross-check (`auto_takeoff.h`).
- **Velocity control:** `VelocityController` (PI, vel error → angle setpoint) exists but
  `update()` is **never called** in flight code.
- **Position hold:** `PositionController` (P+I on flat-earth NED from E7 lat/lon, 4 m/s
  clamp, arrival latch, antimeridian unwrap) — **never called**.
- **RTH:** `ReturnToHome` (CLIMB→NAVIGATE→DESCEND→LANDING, GPS-stale abort) — only home
  *capture* is wired (`main.cpp:3968–3977`); `update()` is **never called**. The iBus
  CH7 RTH trigger sets `gRthRequested`, which nothing consumes.
- **Landing:** `LandingController` (sink-rate PID on ToF derivative: 0.7 m/s cruise →
  0.25 m/s final meter → touchdown detect + 600 ms hold → disarm) **is wired — but only
  as the link-loss failsafe descent** (`main.cpp:6506–6556`), not a pilot-commanded mode.
- **Cascaded 4-tier controller** (`cascaded_controller.h`: pos 20 Hz → vel 50 Hz →
  att 250 Hz → rate 500 Hz, saturation back-propagation): **`step()` never called.**
- **Throttle:** 0–100 % mapped to DShot 48..1800 (of 2047), slew-limited 250 %/s up /
  400 %/s down. **Zero throttle (or invalid IMU) = full motor stop + PID reset, even
  airborne** (`main.cpp:7011–7014`). No air mode, no armed idle.
- **Mixer:** quad-X additive (`main.cpp:7175–7180`), per-axis sign flags, front-motor
  pitch bias ×1.3 for forward CG (`platformio.ini:134`). Per-motor clamp to 0 or
  [48..2047] only — **no desaturation/normalization** when a motor hits a limit.

### Sensor fusion
- **Live estimator:** Euler complementary filter, τ = 0.5 s, accel trusted only when
  |a| ∈ [0.85, 1.15] g (gyro-only otherwise); yaw = gyro integration + tilt-compensated
  mag heading blended in (fusion enabled by default, hard/soft-iron cal via stick
  gesture, NVS-persisted) (`main.cpp:3048–3102`).
- **Gyro filtering:** ICM-20948 hardware DLPF → dynamic notch (2 biquads/axis,
  fundamental + 2nd harmonic). The notch is **motor-command-mapped** (sqrt mapping from
  highest DShot command to expected frequency, 45–120 Hz anchors, A/B validated −5…−6 dB)
  — *not* FFT/SDFT like Betaflight. An eRPM-driven RPM notch exists for bidirectional
  DShot but is compiled out (`FCU_ENABLE_RPM_FILTER=0`).
- **Shadow estimator:** genuine 15-state error-state Kalman filter (p, v, q, gyro bias,
  accel bias; analytic F, sequential scalar updates, innovation gating, accel-tilt
  gravity update, mag yaw, GPS/baro/ToF fusion, divergence recovery), sim-validated by
  an 8-scenario synthetic suite (`ekf_estimator.h`, `src/ekf_sim_test.cpp`). Runs every
  tick but is **shadow-only**: `ENABLE_EXPERIMENTAL_EKF=1`, `..._CONTROL=0`.
- **GPS:** NMEA **GGA only** at 9600 baud — fix/sats/lat/lon/alt. RMC (ground speed,
  course) is an acknowledged TODO (`gps_module.h:19–21`), so **no GPS velocity exists**
  for the unwired velocity loop.
- **Baro:** BMP280 @ ~25 Hz — takeoff cross-check + telemetry only; not blended into
  the altitude-hold loop. **ToF is the sole altitude-hold sensor** (≤ ~4 m).
- **Mag:** AK09916 (inside ICM-20948), tilt-compensated, declination-corrected,
  field-magnitude sanity gate.

### Receiver / input handling
- **ELRS/CRSF** on UART0 @ 420 k (CRC8-validated frames, LQ/RSSI link stats, 2 % stick
  deadband). Map: CH1–4 = AETR, CH5 = arm (hysteresis), CH7/CH8 = camera pan/tilt.
  **CH6 flight-mode switch, RTH, failsafe-clear: deliberately unmapped**
  (`updateFlightModeFromAuxChannels` is a TODO stub, `main.cpp:5979`).
- All RC sources funnel into one internal `ControlPacket` (legacy 32-byte nRF24 layout,
  carries PID gains; sanitized + clamped on receipt) → `processControlPacket()`.
- **Arming:** CH5 high + "safe-boot" latch (throttle held low after first frames) +
  no failsafe + valid IMU. Disarm resets PID integrators.
- **Failsafe:** 350 ms control-link timeout; latched first-cause reason codes (link, Pi
  heartbeat, ToF-when-needed, low battery, 60° tilt, IMU invalid, 30 ms loop overrun).
  Link loss → hold last throttle, then either **sensor-driven auto-land** (ToF +
  LandingController, if IMU ok and tilt < 30°) or legacy linear ramp-down. Failsafe
  clear requires an explicit request *and* safe conditions; motor release goes through
  a soft idle-ramp rather than a step.

### Motors / ESC
- **DShot300 over RMT** (vendored `easy-esc-esp32` + `DShotRMT`), motors on GPIO 39–42,
  refreshed at the 500 Hz flight tick. Bidirectional DShot (eRPM telemetry) supported
  but off pending pull-up hardware. 3 s zero-throttle ESC settle window at boot.
  No DShot commands (beacon etc.), no ESC passthrough in the flight image (lib exists).

### Task / loop architecture (FreeRTOS, ESP32-S3 dual-core)
| Task | Core / prio | Rate | Work |
|---|---|---|---|
| `flightTask` | 1 / 24 | 500 Hz | DShot refresh → failsafe eval → IMU SPI read → notch → attitude → PID → mixer |
| `radioTask` | 1 / 23 | IRQ + 10 ms floor | nRF24 telemetry TX (custom GCS), init retries |
| `sensorTask` | 0 / 8 | 50 Hz | ToF, baro, GPS, Pi UART, battery, observer FSM, LEDs |
| `crsfControlTask` | 0 / 7 | 5 ms | CRSF parse → ControlPacket dispatch, gimbal |
| `loop()` | 1 / 1 | — | status logging only |

Plus: task watchdog on all tasks, loop-overrun failsafe, spinlock-guarded state
snapshots, a single-owner queue for EKF measurements. This split (fast attitude core +
slow navigation/IO core) resembles a miniature PX4 work-queue layout more than
Betaflight's single superloop.

### Autonomy level (wired vs. present)
- **Wired:** angle-mode flight, auto-takeoff, ToF altitude hold, link-loss auto-land,
  home capture, failsafe ladder, camera gimbal.
- **Present but not wired:** velocity control, position hold, RTH execution, LAND mode,
  mode switching on CRSF, 4-tier cascade, EKF-driven control, gain scheduling.
- **Companion computer:** Raspberry Pi UART protocol (heartbeat + confidence-gated AI
  commands HOVER/FORWARD/…/LAND/STOP) that maps to **stick-equivalent nudges**, not
  trajectories — a much simpler contract than MAVLink offboard. Disabled in the build.
- **Missing entirely:** waypoint missions, geofence (stubbed `inGeofence=true`),
  MAVLink/MSP/any standard GCS protocol, onboard blackbox/flight logging.

---

## 3. Closest Market/Existing FCU Comparison

### Betaflight — similarity 5.5 / 10
- **Why similar:** the inner-loop *feature menu* is recognizably Betaflight-derived:
  DShot via timer peripheral, dynamic gyro notch, optional eRPM notch filter,
  D-term lowpass, D-on-measurement, integral windup management, ELRS/CRSF with LQ/RSSI,
  stick deadband, throttle-low arming gate. The ANGLE-mode structure (angle-P outer →
  rate-PID inner feeding an additive quad-X mixer) is exactly BF's ANGLE mode.
- **Why different:** everything that makes Betaflight *Betaflight* is absent: no acro
  rate mode, no rates/expo shaping, no feedforward, no air mode / dynamic idle, no mixer
  desaturation, no TPA, no blackbox, no MSP/CLI/configurator, no OSD. Loop is 500 Hz vs
  BF's 4–8 kHz gyro-synchronous loop; the notch is motor-command-inferred, not
  FFT/SDFT-measured; attitude is Euler complementary, not Mahony quaternion.
- **Matches:** `DynamicNotchFilter.*`, `RpmNotchFilter.*`, `fcu_pid.h` D-path,
  `crsf_receiver.h`, DShot/RMT setup, angle→rate cascade in `updateControlLoop`.
- **Doesn't match:** absence of acro/FF/airmode/desat; `forceMotorStop()` on zero
  throttle; web-server tuning instead of MSP; Arduino/ESP32 platform.

### Cleanflight / Baseflight / MultiWii-era — similarity 6 / 10 (for the *flying* core)
- **Why similar:** the control law actually reaching the motors — Euler complementary
  filter, self-level-only angle mode, ~500 Hz PID, simple additive mix, percent-scaled
  throttle — is sophistication-equivalent to Cleanflight-era ANGLE firmware (with
  better filtering and far better failsafe engineering than that era had).
- **Why different:** no rate mode, no configurator protocol, and conversely this
  firmware's failsafe/state-machine rigor, notch filtering, and DShot are beyond the
  MultiWii generation.
- **Matches:** complementary filter (`main.cpp:3048`), mixer, single packet-driven
  control path. **Doesn't match:** filtering stack is more modern; navigation scaffolding
  far exceeds it.

### ArduPilot (ArduCopter) — similarity 6 / 10 (architecture), ~3 / 10 (active code)
- **Why similar:** the autonomy design language is ArduCopter's, sometimes literally:
  mode names MANUAL/ALT_HOLD/POS_HOLD/RTH/LAND (`flight_modes.h`); RTL implemented as
  climb-to-cruise → navigate-home → descend → hand to lander (`return_to_home.h`) which
  is ArduCopter RTL's exact phasing; first-fix home capture; failsafe escalation with
  latched reasons; battery/tilt/EKF-style arming guards in a central state machine
  (`flight_state_machine.h`); E7 lat/lon convention (the GPS header cites
  Pixhawk/PX4); error-state EKF with innovation gating in the EKF2/EKF3 lineage
  (15-state vs their 22–24); Stabilize-mode control law (angle-P → rate-PID) is also
  ArduCopter's.
- **Why different:** ArduPilot's spine — parameters, MAVLink, missions, dataflash logs,
  EKF-in-the-loop, land detector, multi-sensor redundancy — is missing or inactive here.
  The EKF, cascade, position/velocity/RTH controllers are all disconnected from motors.
  Altitude hold is a single ToF PI, not AC's pos→vel→accel Z cascade.
- **Matches:** `flight_modes.h`, `return_to_home.h`, `failsafe_manager.h`,
  `flight_state_machine.h`, `ekf_estimator.h`, Pi-UART companion concept.
- **Doesn't match:** everything between those modules and the motors.

### PX4 — similarity 5 / 10
- **Why similar:** `gnc::CascadedController` mirrors PX4's mc_pos/mc_att/mc_rate
  controller split including the staged rates (20/50/250/500 Hz), NED frames,
  normalized torque outputs, and saturation back-propagation. The ESKF with sequential
  scalar updates is textbook PX4/ECL practice. Dual-core task partitioning with
  priorities and watchdogs is closer to PX4's executive style than to any hobby
  firmware. The observer FSM is a proto-Commander.
- **Why different:** no uORB-style middleware, no parameter system, no missions, no
  offboard protocol, no logging; and again — the PX4-like parts are shadow/scaffold.
  What actually flies is far simpler than anything PX4 ships.
- **Matches:** `cascaded_controller.h`, `ekf_estimator.h`, `frames.h`/`math3.h`,
  task layout. **Doesn't match:** active control path, ecosystem, redundancy.

### INAV — similarity 6.5 / 10 ← **closest overall**
- **Why similar:** INAV *is* "Cleanflight inner loop + onboard GPS navigation," which is
  precisely this codebase's identity and roadmap: angle-centric flight, ToF/baro/GPS
  sensor mix, ALT_HOLD/POS_HOLD/RTH mode set on RC switches, RTH that climbs first then
  lands, failsafe→auto-land, simpler-than-PX4 estimator philosophy, hobby RC links
  (ELRS) rather than GCS-first operation. The iBus (fallback) path even already maps a
  3-pos mode switch and an RTH switch exactly INAV-style.
- **Why different:** INAV's navigation is mature and *wired* (waypoint missions,
  safehomes, braking modes, fixed-wing support, configurator, blackbox); here the
  equivalent modules exist but aren't connected, and GPS velocity isn't even parsed yet.
  INAV also retains acro mode.
- **Matches:** overall architecture intent, mode taxonomy, sensor strategy, RTH design.
- **Doesn't match:** wiring/maturity of all navigation features; mission capability.

### DJI / Naza-style consumer architecture — similarity 4 / 10
- **Why similar:** the *operator model* is consumer-drone-shaped: always self-leveled
  with a hard 15° envelope, no acro, one-switch arm, auto-takeoff to a target altitude,
  link-loss → autonomous controlled descent (classic DJI behavior), camera pan/tilt
  handled by the FC, GPS hold/RTH as the headline roadmap. The repo's own commit
  history says "DJI-like auto-takeoff."
- **Why different:** DJI's actual stack (vision/optical-flow odometry, dual-redundant
  sensors, mission/tracking software) has no counterpart here; closed-source prevents
  implementation-level comparison; current firmware lacks position hold, the defining
  Naza feature.
- **Matches:** `auto_takeoff.h`, safe-landing failsafe, `camera_gimbal.h`, tilt clamp.
- **Doesn't match:** vision/redundancy/positioning depth.

### Final ranking
1. **Closest: INAV** — same lineage (racing-firmware inner loop + onboard GPS autonomy),
   same mode taxonomy, same RTH shape; this firmware is roughly "INAV before the nav
   code was wired in."
2. **Second closest: ArduPilot (ArduCopter)** — the autonomy scaffolding (modes, RTL,
   failsafe ladder, FSM, ESKF) is ArduCopter-shaped; active control law equals AC
   Stabilize. (Betaflight is a close third for the inner-loop/filtering DNA;
   Cleanflight-era firmware best describes the *currently flying* sophistication.)
3. **Least similar: PX4 and DJI/Naza** — PX4's middleware/ecosystem and DJI's
   vision-based positioning have no active counterpart; both match only in borrowed
   design patterns (PX4) or product intent (DJI).

**Category answer:** not Betaflight-style racing firmware (no acro/airmode/high-rate
loop), not yet ArduPilot/PX4-style autopilot (autonomy not wired), not DJI-grade
(no positioning). It is a **custom hybrid trending toward INAV-style GPS-assisted
firmware**, currently flying at Cleanflight-ANGLE capability with Betaflight-style
filtering and ArduPilot-style safety scaffolding.

---

## 4. Missing features vs. real market FCUs

| Feature (market baseline) | Status here |
|---|---|
| Acro / rate mode | **Not found** (angle-only control path) |
| Air mode / armed idle / dynamic idle | **Not found** — zero throttle stops motors and resets PIDs even airborne (`main.cpp:7011`) |
| Feedforward on setpoints | Not found |
| Rates / expo / setpoint smoothing | Not found (linear stick→15°, 2 % deadband only) |
| Mixer desaturation / normalization | **Not found** (per-motor clamp only) |
| TPA (throttle PID attenuation) | Not found |
| Gyro-synchronous ≥2 kHz loop | Not found (fixed 500 Hz tick) |
| FFT/SDFT-measured dynamic notch | Different design — motor-command-mapped notch (works, but open-loop w.r.t. actual vibration) |
| RPM filter (eRPM-driven) | **Partially implemented** — code present, compiled out pending BDShot pull-ups |
| In-loop EKF state estimation | **Partially implemented** — full ESKF runs in shadow, never controls |
| GPS velocity / course | **Not found** (GGA-only parser; RMC is a marked TODO) |
| Position hold | **Partially implemented** — controller exists, never called |
| RTH | **Partially implemented** — state machine + home capture exist; execution not wired |
| Pilot-commanded LAND mode | **Partially implemented** — lander exists but only as link-loss failsafe |
| Waypoint missions | Not found |
| Geofence | Not found (`inGeofence` hardcoded true) |
| Heading hold / yaw angle mode | Not found (yaw is rate-only) |
| Baro/GPS/ToF blended altitude | **Not found in control** (ToF-only hold; baro is takeoff cross-check; blending exists only inside shadow EKF) |
| Blackbox / onboard flight logging | Not found (serial CSV `[TUNE]` lines only) |
| Standard GCS protocol (MAVLink/MSP/CRSF-param) | Not found (custom nRF24 packets + bench web server) |
| Parameter system | Ad-hoc NVS fields (works, but no schema/export) |
| Sensor redundancy (dual IMU/baro) | Not found (single ICM-20948, single BMP280, single VL53L1X) |
| Land detector (accel/throttle based) | ToF+Vz touchdown detector in lander only |
| Compass health / interference monitor | Field-magnitude gate only |
| ESC telemetry (volts/amps/temp) | Supported via BDShot, currently off |

## 5. Risks in the current implementation

1. **Zero-throttle in flight = motor stop + PID reset** (`main.cpp:7011–7014`). A pilot
   chopping throttle mid-air gets total thrust loss plus slew-limited respool. Every
   market FCU keeps min-spin while armed. Decide the policy (armed idle vs. current)
   before any free flight with a human on the sticks.
2. **No mixer desaturation.** Base throttle can reach raw 1800 while PID adds up to
   ±240×3 axes; `clampMotorRaw` truncates at 2047/48 per motor, silently distorting the
   commanded moment exactly when authority matters most (high throttle + disturbance).
   Betaflight/ArduPilot rescale all motors to preserve the moment.
3. **Angle-only recovery ceiling:** with AngleP=5.0 and the 240 °/s rate clamp, recovery
   authority saturates near ~48° tilt (own comment, `platformio.ini:135–147`); combined
   with the 60° tilt failsafe there is a narrow band where the craft is failing but not
   yet latched.
4. **kd = 0** on the rate loops: no damping term; the craft relies on P/I alone. Known
   and deliberate, but it caps achievable stiffness and makes oscillation tuning harder.
5. **ToF-only altitude hold:** VL53L1X range (~4 m, sunlight/surface sensitive) bounds
   every altitude feature; baro is not blended in the live loop. ToF failure mid-hold is
   a latched failsafe (good) but there is no graceful baro fallback in the hold itself.
6. **Complementary filter limits:** sustained horizontal acceleration biases the
   gravity reference (accel gate only rejects |a| outside 0.85–1.15 g, not coordinated
   acceleration); Euler form is fine inside ±15° but degrades if the envelope is raised.
7. **PositionController frame shortcut:** it explicitly treats NED ≡ body ("assumes the
   drone is yaw-aligned with home heading", `position_controller.h:121–124`). Wiring
   POS_HOLD/RTH without inserting the yaw rotation will produce wrong-direction
   corrections on any non-north heading. (The cascade's `toBodyTilt()` does it right.)
8. **GPS data gap:** 9600-baud GGA-only at low rate, no velocity — insufficient for the
   velocity loop it is meant to feed; this is the single biggest blocker to activating
   POS_HOLD (the EKF-velocity seam `FCU_USE_EKF_VELOCITY` is the planned fix).
9. **Front-pitch mix bias ×1.3** raises effective pitch-loop gain on the front pair;
   any future pitch retune must re-verify with the bias active (and the bias is
   web-adjustable in flight).
10. **Throttle slew 250 %/s** adds up to ~400 ms to reach full thrust from zero —
    relevant to escape maneuvers; combined with risk #1 it makes a low-altitude
    throttle chop unrecoverable.
11. **Bench web server in the flight repo:** properly gated (separate env, failsafes
    disabled only there, loud warnings, auth token) — keep it that way; never extend
    `esp32-s3-mini-pidweb` into a flyable image.
12. **No onboard logging:** post-incident analysis depends entirely on USB serial being
    attached; a blackbox-style flash/SD log would change tuning and debugging economics.

## 6. What to add to get closer to each system

### Toward Betaflight (racing/manual performance)
1. Acro (rate) mode branch in `updateControlLoop` + mode switch (the FSM already names MANUAL vs STABILIZE).
2. Air mode / armed idle: replace zero-throttle motor stop with min-spin + active PIDs while armed (with a true disarm path).
3. Mixer desaturation (shift/scale all motors to preserve commanded moments; the cascade's `notifyMotorSaturation` hook is already the right shape).
4. Feedforward from setpoint derivative + rates/expo shaping + setpoint smoothing.
5. Raise loop rate (gyro-data-ready driven; the S3 + 7 MHz SPI can do 1–2 kHz) and add a proper gyro LPF chain.
6. Replace/augment the command-mapped notch with SDFT-based tracking, or finish BDShot pull-ups and enable the existing RPM filter.
7. Blackbox logging (flash ring buffer) and an MSP or CLI configuration channel.

### Toward PX4 / ArduPilot (autopilot)
1. Flight-validate the shadow EKF (the `[EKF]` shadow-log flight you already planned), then flip the cascade/EKF seams on incrementally.
2. Parse GPS velocity (RMC/VTG or a uBlox UBX driver at >1 Hz) — or commit to the EKF-velocity seam — and only then wire `VelocityController`.
3. Wire the existing PositionController → VelocityController → angle-setpoint chain into POS_HOLD, with the missing yaw rotation fixed.
4. Promote the FSM from observer to authority (single dispatcher in `updateControlLoop` keyed off `gFlightSm.state()`), absorbing today's scattered `allowFlight`/ATK/link-loss conditionals.
5. Wire RTH end-to-end (trigger → climb → navigate → descend → existing lander), including the battery-RTH threshold already present in the FSM inputs.
6. Mission/waypoint storage + geofence to replace the stubs; a real parameter table; MAVLink (even just heartbeat+attitude+RC-override) so standard GCS tools work.
7. Multi-sensor altitude fusion (baro+ToF+GPS via the EKF) instead of raw ToF in the hold loop; add a land detector independent of ToF.

### Toward INAV (GPS-assisted RC flying)
1. Map CH6/CH9+ on CRSF to the mode switch (`updateFlightModeFromAuxChannels` stub is the designated place) — MANUAL / ALT_HOLD / POS_HOLD + RTH switch.
2. Make ALT_HOLD pilot-commandable (throttle stick → climb-rate around hover, as `flight_modes.h` already documents but the code doesn't implement).
3. Wire POS_HOLD with stick-commanded velocity offsets (INAV's "cruise" feel) on top of items from the PX4/AP list.
4. Failsafe escalation policy: link loss → (GPS ok? RTH : land-in-place) — the modules and reason codes all exist; it's pure wiring.
5. Safehome-style RTH altitudes per home point; RTH abort on stick input (manual-override detection already exists).

## 7. Test before any real flight (sim / props off / tethered)

**Props-off bench (existing envs):**
1. `fcu_bench_test` — motor order/direction + simulated mixer response to hand tilts (already automated).
2. `fcu_motor_fft_test` — re-verify notch tracking at the new 1800 throttle ceiling (the ini comment requires this above ~1850; you're at 1800 — re-run anyway after any prop/mount change).
3. Arming-gate matrix: CH5 high without safe-boot latch, throttle-high at boot, failsafe-latched arming attempts — confirm every `BLOCKED_*` path refuses.
4. Failsafe drill: kill the TX mid "flight" (props off, throttle up): verify 350 ms latch → hold → LandingController engage (with ToF spoofed/bench target) → soft-release ramp → disarm; verify failsafe-clear is refused until conditions are safe.
5. IMU-loss drill: disconnect IMU SPI mid-run → 8-tick grace → motor stop + `kFailsafeImuInvalid`.
6. Loop-overrun injection (debug delay in flightTask) → confirm the 30 ms overrun failsafe latches.
7. Auto-takeoff dry run, props off: confirm liftoff-timeout abort fires at ~3 s since ToF won't rise (this validates the "props off / stuck" abort exactly as designed).
8. Zero-throttle policy review on the bench: deliberately chop throttle at mid-stick and watch motor stop + PID reset; decide if that's acceptable before a human flies it.

**Simulation (existing + recommended):**
9. `native_ekf_sim` / `ekf_sim` — keep the 8-scenario suite green after any estimator edit (it has already caught a real init-sign bug).
10. Add a closed-loop sim scenario for the *active* stack (complementary filter + angle/rate PID + mixer against a point-mass/inertia model) before raising kd from 0 — the S9 velocity scenario shows the harness can do this.
11. Sim the PositionController yaw-rotation fix (command POS_HOLD at 90° heading; verify correction direction) before ever wiring it.

**Tethered / guarded first flights:**
12. One hover flight with `FCU_EKF_SHADOW_LOG=1` → offline EKF-vs-complementary comparison (your stated gate for any EKF-in-the-loop work).
13. `[TUNE]`/`[TUNE_P]` capture at hover for rate-loop step response; only then add D.
14. Alt-hold step tests on a tether over a hard surface (ToF-friendly), including a deliberate ToF occlusion to confirm the latched failsafe behaves.
15. Battery-sag failsafe check with a loaded pack near the 10.5 V threshold.
16. Link-loss auto-land, tethered, from ~2 m: confirm DESCEND→APPROACH→TOUCHDOWN→disarm and the bounce-recovery branch.

---
*Generated from direct source inspection of `src/main.cpp` (8,651 lines), all 42 headers in
`include/`, `platformio.ini`, and `lib/` on branch `rtos-refactor`. No code was modified.*
