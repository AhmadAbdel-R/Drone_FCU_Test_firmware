# ESP32-S3 FCU Firmware Analysis

Date: 2026-06-09  
Scope: project firmware under `src/` and `include/`, local EasyESC/DShotRMT libraries under `lib/`, and a local depth-1 Betaflight clone under `betaflight/`. No firmware or library source files were modified for this analysis.

## SETUP

Target inferred from `platformio.ini`: ESP32-S3 DevKitM-1 (`board = esp32-s3-devkitm-1`) using the Arduino framework on top of ESP-IDF/FreeRTOS (`framework = arduino`). The primary environment is `[env:esp32-s3-mini]`.

Confirmed project locations:

| Item | Location | Notes |
|---|---|---|
| Main firmware source | `src/` | Primary build includes `src/*` except `test_main.cpp`, `pid_webserver_enabled.cpp`, and `ekf_sim_test.cpp`; `src/main.cpp` is the main flight firmware. |
| Firmware headers/modules | `include/` | Control, failsafe, EKF, receiver parsers, navigation helpers, etc. |
| EasyESC library | `lib/easy-esc-esp32/` | High-level ESC wrapper used by `src/main.cpp`. |
| DShotRMT dependency | `lib/DShotRMT/` | Vendored local DShot/RMT driver used by EasyESC. |
| Betaflight clone | `betaflight/` | `git clone --depth 1 https://github.com/betaflight/betaflight` was run because the directory was missing. |

Relevant Betaflight areas used:

| Area | Betaflight paths |
|---|---|
| Generic DShot | `betaflight/src/main/drivers/dshot.c`, `dshot.h`, `dshot_command.c`, `dshot_command.h`, `dshot_bitbang*` |
| Generic motor abstraction | `betaflight/src/main/drivers/motor.c`, `motor.h`, `motor_impl.h`, `motor_types.h` |
| ESP32 RMT DShot | `betaflight/src/platform/ESP32/dshot_esp32.c`, `pwm_motor_esp32.c` |
| Mixer and motor policy | `betaflight/src/main/flight/mixer.c`, `mixer_init.c` |
| Failsafe and arming | `betaflight/src/main/flight/failsafe.c`, `failsafe.h`, `betaflight/src/main/fc/core.c`, `fc/rc_controls.c` |

## PASS 1 - Bug And Vulnerability Scan Of My Code

### Findings

| ID | Location | Severity | Finding | Why it is a problem | Fix |
|---|---|---:|---|---|---|
| F1 | `src/main.cpp:5910-5925`, `src/main.cpp:5799-5859`, `include/crsf_receiver.h:277-291` | Critical | CRSF receiver-reported failsafe is checked only after a new RC frame is dispatched. | ELRS/CRSF can keep serial frames arriving while uplink LQ drops to 0. `crsfControlTask()` calls `buildAndDispatchCrsfControlPacket()` on `gotFrame` before evaluating `!gCrsf.failsafeActive()`, so `acceptControlPacketFromRadio()` refreshes `gState.control.lastPacketMs` and can keep the flight-side link timeout from latching promptly. Roll/pitch/throttle can continue from receiver failsafe or stale channel values; only yaw is cleared later at `src/main.cpp:5933-5936`. | Compute `linkUp` before dispatch. If `gCrsf.failsafeActive()` or link inactive, do not dispatch the RC packet. Instead set `gState.control.linkActive=false`, latch failsafe, and optionally inject one explicit zero-throttle failsafe packet. |
| F2 | `include/crsf_receiver.h:109-145`, `include/crsf_receiver.h:248-260` | High | CRSF RC decoder does not enforce the fixed RC payload length before decoding 22 payload bytes. | The frame parser accepts any CRSF length from 2 to 62 if CRC is valid. If a short type `0x16` RC frame arrives, `decodeRcChannels()` still reads 22 bytes starting at `buffer_[3]`. This stays inside the 64-byte local buffer, but reads stale bytes from prior frames and can corrupt all channel values from external input. | Require `expectedLen_ == 24` before `decodeRcChannels()` (`type + 22 payload + crc`). Similarly require `expectedLen_ == 12` for link statistics rather than `>= 12`. Reject and count malformed typed frames. |
| F3 | `src/main.cpp:3666-3822` | High | GPS NMEA parsing trusts unchecked, unauthenticated, weakly parsed GGA data for GPS fix, home capture, and EKF origin/update. | `splitNmeaFields()` strips at `*` but never verifies the NMEA checksum. `parseNmeaCoordE7()` uses `atof()` and treats malformed or empty coordinates as 0, and any hemisphere other than `S/W` as positive. `parseGpsLine()` sets `hasFix` from `fixQuality > 0`, captures RTH home from an undebounced first fix at `src/main.cpp:3772-3787`, and can set EKF origin from that same data at `src/main.cpp:3791-3818`. UART noise or spoofed/malformed GPS sentences can create a false home/origin, including `0,0`. | Verify NMEA XOR checksum before parsing. Use strict numeric parsing with end-pointer checks. Validate latitude/longitude ranges, hemisphere tokens, finite altitude, minimum satellites, HDOP, and acceptable fix quality before setting `hasFix`, home, or EKF origin. Remove the undebounced first-fix home/origin fallback for RTH. |
| F4 | `src/main.cpp:3517-3518`, `src/main.cpp:3637-3638`, `src/main.cpp:3796-3818`, `src/main.cpp:6440-6456`, `include/ekf_estimator.h:169-210`, `include/ekf_estimator.h:232-315`, `include/ekf_estimator.h:361-362` | High | `gEkf` is mutated from multiple FreeRTOS tasks without a mutex or single-owner model. | Sensor task updates baro, ToF, GPS, and EKF origin; flight task predicts IMU and updates magnetometer at 500 Hz. The EKF mutates shared state and a 15x15 covariance matrix. This is a C++ data race and can corrupt estimator state. In the current build EKF is shadow-only for control, but it is enabled by default and the risk becomes flight-critical if `ENABLE_EXPERIMENTAL_EKF_CONTROL=1` or `FCU_USE_EKF_VELOCITY=1`. | Make the EKF single-owner. Prefer queueing sensor measurements into the flight task and applying all EKF predict/update operations there. If that is not feasible, protect every `gEkf` method, including snapshots, with a dedicated mutex. |
| F5 | `src/main.cpp:1895`, `src/main.cpp:6032`, `src/main.cpp:2834`, `src/main.cpp:2863`, `src/main.cpp:4481`, `src/main.cpp:4572`, `src/main.cpp:4663`, `src/main.cpp:8171-8173` | Medium | `gFlightSm` is read and written across tasks without synchronization. | `updateFlightStateMachine()` mutates the object from the sensor task; webserver, control, logging, and telemetry paths read state/time/count elsewhere. This is a C++ data race on a safety-relevant state gate used for bench PID writes and motor spin decisions. | Keep the FSM on one task and publish an atomic state snapshot, or guard every `gFlightSm` method with a mutex. Use the atomic snapshot for safety gating. |
| F6 | `platformio.ini:233-245`, `src/pid_webserver_enabled.cpp:342-351`, `src/pid_webserver_enabled.cpp:382-399`, `src/pid_webserver_enabled.cpp:553-579` | High for `esp32-s3-mini-pidweb` env | PID webserver environment has hard-coded WiFi credentials, disables failsafes, and exposes unauthenticated mutation/motor-spin endpoints. | The alternate PID web build adds `ENABLE_PID_WEBSERVER=1`, `FCU_DISABLE_FAILSAFES=1`, and committed SSID/password values. HTTP endpoints allow PID/mix mutation, save/revert/reset/calibration, and `POST /api/motor/spin` with no authentication. Anyone on that network can change flight-critical settings or request motor spins while the bench gates consider it safe. | Move WiFi credentials to ignored local config or environment variables. Add authentication/token checks. Require a physical safety condition for motor endpoints. Make the PID web build visibly non-flightable and keep failsafe-disabling flags out of reusable config. |
| F7 | `include/ibus_receiver.h:55-95` | Low, disabled in current env | iBUS parser drains `while (serial_->available() > 0)` without a per-call byte budget. | Current primary env sets `USE_FLYSKY_IBUS_CONTROL=0`, but if enabled a UART flood or large backlog can make one poll drain unbounded bytes in one task iteration. Other parsers have explicit budgets. | Add a max bytes or max frames per poll, similar to the CRSF/GPS/Pi parser guard patterns. |
| F8 | `include/position_controller.h:90-91` | Low | Position delta uses `int32_t` subtraction for E7 latitude/longitude differences. | Bad targets, distant coordinates, or antimeridian cases can overflow signed 32-bit subtraction before converting to meters. Position control is not the dominant current motor path, but `FCU_ENABLE_POSITION_CTRL=1` is configured. | Use `int64_t` for `dLatE7`/`dLonE7`, validate local radius before computing, and reject antimeridian/remote targets for this local controller. |
| F9 | `src/main.cpp:1940`, `src/main.cpp:6366-6457`, `include/ekf_estimator.h:203-207` | Medium | EKF covariance propagation runs in the 500 Hz flight loop and allocates two 15x15 float matrices on stack each predict. | `flightTask` stack is 8192 bytes. `predictIMU()` uses large stack arrays and matrix multiplies inside a 2 ms real-time loop. It currently built, but worst-case loop time and stack high-water are not proven in the report data. | Measure flight task worst-case duration and stack high-water with EKF enabled. Consider single-owner lower-rate EKF update, static buffers, or disabling shadow EKF until timing is characterized. |
| F10 | `src/main.cpp:4725-4744` | Low | Telemetry state records requested motor raw values before ESC write success is known. | `gState.motorRaw = raw` happens before `spinRaw()` return values are checked. Telemetry/logging can report commanded motor values even when one or more ESC writes failed. | Track commanded and accepted motor outputs separately, or update `gState.motorRaw` after successful writes and include per-motor write status in telemetry. |
| F11 | `src/main.cpp:1763-1780`, `src/main.cpp:5818`, `src/main.cpp:6739-6742` | Low | Yaw side channels are `volatile int8_t` shared between control task and flight task. | `volatile` does not make C++ cross-task access data-race-free. A single-byte ESP32 access is likely physically atomic, but the language-level race remains. | Use `std::atomic<int8_t>` or fold yaw into the mutex-protected control packet path. |
| F12 | `include/autonomy_uart.h:445-451` | Low, disabled in current env | Autonomy packet numeric fields use `atol()` and `atof()` with partial/empty-field acceptance. | Checksums are present, and `FCU_ENABLE_AUTONOMY_UART=0` in the primary build, but if enabled strings such as partial numbers can be accepted as valid commands. | Use `strtol()`/`strtof()` with end-pointer validation and reject empty, partial, or non-finite fields. |

### ISR And Watchdog Notes

The nRF IRQ ISRs (`src/main.cpp:1984-1991`, `src/main.cpp:1998-2005`) do the right kind of work for an ISR: notify a task and yield. I did not find heavy work in those ISRs. The main watchdog gap is not a missing subscription, but the combination of several timing-sensitive workloads inside the 500 Hz flight loop: IMU, EKF predict, filters, motor writes, and serial logging/backpressure logic.

## PASS 2 - Static Analysis

### Build

Command run:

```powershell
.\.venv\Scripts\platformio.exe run -e esp32-s3-mini
```

Result: success. Firmware built for `esp32-s3-mini`.

Build metrics reported:

| Metric | Value |
|---|---:|
| RAM | 29,876 bytes used of 327,680, 9.1% |
| Flash | 512,303 bytes used of 3,145,728, 16.3% |

Compiler warnings seen:

| Location | Summary | Assessment |
|---|---|---|
| `lib/DShotRMT/src/DShotRMT.cpp:637`, `lib/DShotRMT/src/DShotRMT.cpp:653` | GCC warns that `IRAM_ATTR` section attributes conflict between declarations and definitions. | Library issue, not my firmware source. Included in EasyESC/DShotRMT risk notes below. |

### cppcheck

Command run:

```powershell
.\.venv\Scripts\platformio.exe check -e esp32-s3-mini
```

Result: cppcheck installed through PlatformIO and ran, but failed.

Key output:

| Warning | Assessment |
|---|---|
| `src/main.cpp:1990 [high:error] syntax error [syntaxError]` | Tool parse limitation at `portYIELD_FROM_ISR()` inside a valid ESP-IDF ISR macro. The firmware compiler accepted this code. Not merged as a real defect. |
| `src/pid_webserver.cpp` unused functions, `src/pid_webserver_enabled.cpp` unused functions | Low-value source-filter/build-variant noise. The PID webserver risk is captured as F6 for the enabled env, not as unused-code defects. |

### clang-tidy

The local machine had no `clang-tidy` on PATH. I installed PlatformIO's `platformio/tool-clangtidy`, copied its suffixless Windows binary to a temp `clang-tidy.exe`, and generated `compile_commands.json` with:

```powershell
.\.venv\Scripts\platformio.exe run -e esp32-s3-mini -t compiledb
```

Target-aware clang-tidy failed before analysis because the generated ESP32-S3 compile commands contain Xtensa GCC-only flags and incomplete clang-compatible standard include discovery:

- unknown arguments: `-mlongcalls`, `-fno-tree-switch-conversion`, `-fstrict-volatile-bitfields`, `-mdisable-hardware-atomics`
- missing standard headers such as `stdbool.h`, `array`, `cmath`, `stdint.h`

A host-style fallback over `src/BiquadFilter.cpp`, `src/DynamicNotchFilter.cpp`, and `src/RpmNotchFilter.cpp` also failed because clang rejected the Xtensa C library headers. No actionable clang-tidy diagnostics were produced, so none were merged into Pass 1.

## PASS 3 - Compare Against Betaflight

### Motor Output And DShot Differences

| Topic | My firmware / EasyESC | Betaflight reference | Classification |
|---|---|---|---|
| Motor abstraction | Four independent `esc::EasyEscMotor` objects are created in `src/main.cpp:1795-1802`. `applyMotorOutputs()` sequentially calls `spinRaw()` for motors 0..3 at `src/main.cpp:4738-4741`. | Betaflight uses a `motorDevice` vtable and `motorWriteAll()` writes all motor values, then calls one `updateComplete()` barrier (`betaflight/src/main/drivers/motor.c:75-123`). | Deliberate simplification, with a timing-risk edge. Your channels are queued sequentially; Betaflight has an explicit "write all, then start/update" phase. |
| ESP32 DShot timing | DShotRMT uses ESP-IDF RMT at 8 MHz, DShot150/300/600/1200 timing tables, and queues nonblocking `rmt_transmit()` per motor (`lib/DShotRMT/src/dshot_definitions.h:74-123`, `lib/DShotRMT/src/dshot_init.cpp:23-46`, `lib/DShotRMT/src/DShotRMT.cpp:420-448`). | Betaflight's ESP32 driver uses low-level RMT register access at 20 MHz, preloads RMT memory per motor, then starts enabled channels from `dshotUpdateComplete()` (`betaflight/src/platform/ESP32/dshot_esp32.c:52-86`, `181-228`, `284-375`). | Valid design choice for Arduino/IDF portability, but Betaflight's model gives more explicit launch control and overlap prevention. |
| DMA/group update | EasyESC/DShotRMT has no group DMA or group latch. Each motor owns one RMT channel and starts when `spinRaw()` calls `sendThrottle()`. | Betaflight's STM32/AT32/X32 paths use timer/DMA/bitbang variants, and even ESP32 has a vtable update barrier. Generic motor code defers transmission to `updateComplete()`. | Missing feature, not necessarily a bug at 500 Hz DShot300. It can add motor-to-motor phase skew and makes exact output timing harder to reason about. |
| Previous-frame overlap | DShotRMT enforces a per-motor frame interval and returns success with `DSHOT_NONE` if called too soon (`lib/DShotRMT/src/DShotRMT.cpp:401-425`). EasyESC treats any successful result as OK. | Betaflight's ESP32 `dshotUpdateComplete()` waits up to 500 us for prior TX completion before restarting a channel (`betaflight/src/platform/ESP32/dshot_esp32.c:212-225`). | Library bug/risk. Skipped frames look successful to firmware. At 500 Hz DShot300 this is unlikely in normal throttle updates, but it is a problem for command repeats. |
| DShot commands | EasyESC exposes direction/save/custom through DShotRMT, but DShotRMT repeats settings/stop commands at 5 us intervals (`lib/DShotRMT/src/dshot_definitions.h:37-40`, `lib/DShotRMT/src/DShotRMT.cpp:118-148`, `281-299`). The same driver requires roughly 126 us between DShot300 frames. | Betaflight queues inline DShot commands and spaces repeats around normal motor output timing; settings-like commands repeat 10 times with 1 ms timing (`betaflight/src/main/drivers/dshot_command.c:185-285`, `297-352`). | Bug in EasyESC/DShotRMT command path. Most repeats are silently skipped because the interval guard returns success without transmitting. |
| Bidirectional DShot | Build flag `FCU_DSHOT_BIDIR` defaults to 0 (`src/main.cpp:1277-1307`). If enabled, EasyESC polls `getTelemetry()` before sending each motor (`lib/easy-esc-esp32/src/esc_dshot_output.cpp:493-534`). | Betaflight integrates telemetry request bits, waits/decodes around motor update, blocks arming when DShot telemetry is configured but inactive (`betaflight/src/main/drivers/dshot.c:186-199`, `266-287`; `betaflight/src/main/fc/core.c:420-425`). | Deliberate simplification while BDShot is off. If enabling RPM filters, add arming checks and clear telemetry health handling similar to Betaflight. |
| Extended telemetry | DShotRMT contains a full telemetry frame decoder, but `_processFullTelemetryFrame()` is not called anywhere (`lib/DShotRMT/src/DShotRMT.cpp:653`; only declaration/reference found). EasyESC reports eRPM/RPM from `getTelemetry()`. | Betaflight decodes eRPM and extended DShot telemetry types into per-motor telemetry state (`betaflight/src/main/drivers/dshot.c:223-287`, `315-340`). | Missing feature or unfinished code in EasyESC/DShotRMT. Fine if only eRPM is expected; misleading if voltage/current/temp are expected. |
| Arming gates | My code gates motor authorization through CRSF arm switch, safe boot throttle hold, `gState.control.failsafeActive`, RPM filter health when enabled, and custom failsafe logic (`src/main.cpp:4263`, `6480-6890`). | Betaflight has many arming disabled reasons: DShot streaming readiness, RX switch recovery, throttle low, angle, calibration, GPS rescue, DShot telemetry, CPU load, etc. (`betaflight/src/main/fc/core.c:318-425`). | Deliberate simplification, with gaps. The most urgent gap is CRSF receiver failsafe dispatch (F1). |
| Disarm handling | My code can force motor stop and also soft-release for controlled link loss. `applyMotorOutputs()` can re-arm an EasyESC motor if it unexpectedly disarms (`src/main.cpp:4694-4744`). | Betaflight centralizes `disarm()`, clears ARMED, logs, resets crashflip/direction, and writes disarmed motor output through the mixer path (`betaflight/src/main/fc/core.c:492-542`; `betaflight/src/main/flight/mixer.c:484-488`). | Valid custom design, but re-arming inside the output function should remain very tightly bounded and logged as a fault, not normal recovery. |
| Failsafe stages | My code has a custom failsafe manager plus controlled link-loss safe landing/soft release, but CRSF LQ=0 can still refresh packets before latching (F1). | Betaflight has staged RX loss handling: Stage 1 delay, low-throttle just-disarm, auto land/drop/GPS rescue, recovery delay, then disarm (`betaflight/src/main/flight/failsafe.c:74-83`, `230-245`, `263-391`). | Custom design. Not a direct bug, but Betaflight's separation between "RX data valid" and "failsafe active" highlights the F1 bug. |
| Mixer saturation | My mixer clamps each motor independently after applying roll/pitch/yaw (`src/main.cpp:6878-6890`). | Betaflight normalizes motor mixes, constrains throttle to avoid clipping, supports airmode transitions, dynamic idle, RPM limiter, and failsafe-specific DShot reserved-range handling (`betaflight/src/main/flight/mixer.c:460-488`, `556-588`, `676-837`). | Deliberate simplification. It can lose attitude authority at saturation; not a security bug, but important for flight quality and edge handling. |

### Betaflight Edge Cases Not Currently Matched

- No equivalent arming-disabled flag model for "DShot telemetry configured but inactive", "RX just recovered with arm switch on", "CPU load/late tasks", or "calibration in progress".
- No centralized DShot command queue synchronized to the PID/motor loop.
- No all-motor update barrier inside the flight firmware's output call.
- No equivalent mixer normalization/airmode/dynamic-idle anti-saturation logic.
- BDShot is documented but off by default; RPM filter warns that it cannot work without BDShot (`src/main.cpp:7762-7766`).

## PASS 4 - EasyESC Library Analysis

### Architecture Overview

| Module | Responsibility |
|---|---|
| `lib/easy-esc-esp32/src/easy_esc.h/.cpp` | User-facing API: `EasyEsc`, `EasyEscMotor`, current-monitor helpers, pin suitability checks, single/multi-motor convenience wrappers. |
| `lib/easy-esc-esp32/src/esc_controller.h/.cpp` | Mid-level controller: config translation, arm/disarm, raw/percent throttle, current sampling, passthrough plumbing. |
| `lib/easy-esc-esp32/src/esc_dshot_output.h/.cpp` | Owns up to 4 `DShotRMT` objects, validates DShot config, clamps raw throttle, sends throttle, services timeout/refresh, polls telemetry. |
| `lib/easy-esc-esp32/src/esc_passthrough.h/.cpp` | Partial BLHeli/4-way style one-wire passthrough scaffold. It can suspend a motor driver and bit-bang transactions, but has TODOs for full protocol semantics. |
| `lib/DShotRMT/src/*` | Low-level ESP32 RMT DShot frame generation, optional bidirectional DShot eRPM receive, DShot command helpers, timing constants. |

Your firmware currently uses four `EasyEscMotor` instances, each wrapping an `EasyEsc`/`EscController` with one motor. That works, but it prevents the library from having any cross-motor scheduling awareness.

### Protocol Support

| Protocol | Supported? | How signal is generated |
|---|---:|---|
| PWM | No | No PWM/LEDC/MCPWM output path in EasyESC. |
| Oneshot | No | No Oneshot timing or output path. |
| Multishot | No | No Multishot timing or output path. |
| DShot150/300/600/1200 | Yes | DShotRMT uses ESP-IDF RMT TX with a bytes encoder and mode-specific timing tables (`lib/DShotRMT/src/dshot_definitions.h:74-123`, `lib/DShotRMT/src/dshot_init.cpp:77-104`). |
| Bidirectional DShot eRPM | Partial | Optional RMT RX on same GPIO; TX done callback starts receive, RX done callback records symbols, foreground `getTelemetry()` decodes eRPM (`lib/DShotRMT/src/DShotRMT.cpp:151-214`, `729-780`). |
| Extended DShot telemetry | Incomplete | Decoder function exists but is not called. EasyESC telemetry fields for voltage/current/temp should not be assumed valid from this path. |
| BLHeli passthrough | Partial scaffold | `EscPassthrough` bit-bangs one-wire transactions, but comments mark the full BLHeli_S command flow, handshake, erase semantics, response length, and checksum validation as incomplete (`lib/easy-esc-esp32/src/esc_passthrough.cpp:131-177`, `249-306`). |

### Timing And Interrupt Model

- Normal throttle update: `EasyEscMotor::spinRaw()` -> `EasyEsc::setMotorRaw()` -> `EscDshotOutput::sendMotorRaw()` -> `DShotRMT::sendThrottle()` -> nonblocking `rmt_transmit()` (`lib/easy-esc-esp32/src/easy_esc.cpp:539-542`, `lib/easy-esc-esp32/src/esc_dshot_output.cpp:521-548`, `lib/DShotRMT/src/DShotRMT.cpp:76-91`, `420-448`).
- Periodic service: `EasyEscMotor::update()` calls `EscDshotOutput::service()`, which enforces signal timeout and periodic refresh while armed (`lib/easy-esc-esp32/src/easy_esc.cpp:519-521`, `lib/easy-esc-esp32/src/esc_dshot_output.cpp:230-254`).
- BDShot ISR path: TX done callback starts RMT receive; RX done callback stores a symbol count and atomic flags. Decode occurs later in `getTelemetry()`, except the unused full-telemetry helper.
- DShotRMT frame guard: `_frame_timer_us` is based on the DShot frame duration plus padding and doubled for BDShot (`lib/DShotRMT/src/DShotRMT.cpp:401-416`). If a caller sends too soon, `_sendPacket()` returns success with `DSHOT_NONE` without transmitting (`lib/DShotRMT/src/DShotRMT.cpp:420-425`).

### Blocking Calls

| Location | Blocking behavior | Risk |
|---|---|---|
| `lib/DShotRMT/src/DShotRMT.cpp:281-299` | Repeated DShot commands use `delayMicroseconds()`. | Short, but the configured 5 us repeat delay is too short for the driver's own frame interval, so repeats are mostly skipped. |
| `lib/easy-esc-esp32/src/esc_passthrough.cpp:207-246`, `303-320` | Bit-banged one-wire passthrough uses microsecond delays in loops. | Do not call from flight loop or while motors are flight-enabled. |
| `lib/easy-esc-esp32/src/easy_esc.cpp:329-351` | Current zero calibration loops over samples and `delay(sampleDelayMs)`. | Bench-only. Do not call from real-time tasks. |

### EasyESC / DShotRMT Risks And Bugs

| Location | Severity | Issue | Fix |
|---|---:|---|---|
| `lib/DShotRMT/src/dshot_definitions.h:37-40`, `lib/DShotRMT/src/DShotRMT.cpp:118-148`, `281-299`, `420-425` | High | DShot command repeat delay is 5 us while the frame interval guard for DShot300 is about 126 us. Most repeated stop/direction/save frames are skipped but reported as successful. | Use a command repeat interval at or above the active frame interval, or integrate command repeats into the normal motor update loop like Betaflight. Treat `DSHOT_NONE` as "not transmitted", not generic success. |
| `lib/easy-esc-esp32/src/esc_dshot_output.cpp:521-548` | Medium | EasyESC does not distinguish successful transmit from DShotRMT's "skipped due interval" success code. | Have EasyESC inspect `result_code` and optionally count skipped frames. Expose transmit/skip/fail telemetry. |
| `lib/easy-esc-esp32/src/esc_dshot_output.cpp:167-200`, `551-570` | Medium | Multi-motor `setAllRaw()` and stored refresh send motors sequentially; no group update barrier. | Add a group preload/commit API for RMT if exact simultaneous launch matters. |
| `lib/DShotRMT/src/DShotRMT.cpp:653-727` | Medium | Full telemetry decode function is unreachable. | Call it from the RX completion path or remove/mark unsupported. Keep heavy decode out of ISR if enabled. |
| `lib/DShotRMT/src/DShotRMT.cpp:637`, `653` | Low | Build warnings for conflicting `IRAM_ATTR` section attributes between declarations and definitions. | Make declaration and definition attributes identical, or apply `IRAM_ATTR` consistently in one place. |
| `lib/easy-esc-esp32/src/esc_passthrough.cpp:131-177`, `283-306` | Low/Medium | Passthrough has TODOs for handshake, response validation, erase semantics, and checksums. | Keep passthrough bench-only until full BLHeli/4-way protocol handling is implemented and tested. |

### Clean Integration Plan

1. Keep EasyESC/DShotRMT isolated behind one firmware motor-output facade. Do not let flight code call DShot commands, passthrough, or calibration directly.
2. Prefer one 4-motor `EscController`/`EasyEsc` instance over four independent `EasyEscMotor` wrappers if you add group scheduling or shared telemetry health.
3. Add a motor output health structure with per-motor fields: last commanded raw, last transmitted raw, skipped-frame count, failed-send count, armed state, and telemetry freshness.
4. Make DShot command sends asynchronous and loop-synchronized. Direction/save/beacon/extended telemetry commands should be queued outside the flight loop and emitted at safe intervals.
5. Keep passthrough and current calibration disabled whenever the flight state is not bench-idle, and require explicit physical safety gating before enabling them.
6. If BDShot/RPM filtering is enabled, add a pre-arm check that each motor has fresh valid eRPM for a short window, matching Betaflight's "DShot telemetry active" arming gate in spirit.

## Prioritized Action List

| Priority | Issue | Severity | Effort | Action |
|---:|---|---:|---:|---|
| 1 | CRSF receiver failsafe dispatch ordering (F1) | Critical | Low | Gate `buildAndDispatchCrsfControlPacket()` on `linkUp` computed before dispatch. Treat LQ=0 as immediate control-link loss. |
| 2 | CRSF RC fixed-length validation (F2) | High | Low | Require exact RC/link-statistics frame lengths before decoding typed payloads. |
| 3 | GPS checksum and strict validation (F3) | High | Medium | Add NMEA checksum, strict numeric parsing, range/quality gates, and remove undebounced RTH/EKF origin fallback. |
| 4 | EKF cross-task data race (F4) | High | Medium | Move all EKF updates to one owning task or add a dedicated EKF mutex around every method call. |
| 5 | DShotRMT repeated command interval bug | High | Medium | Fix repeat timing and distinguish skipped frames from successful transmissions. |
| 6 | PID webserver credentials/auth/failsafe-disabled env (F6) | High for pidweb | Low/Medium | Remove committed secrets, add auth, add physical motor safety gating, and isolate bench builds. |
| 7 | FlightStateMachine data race (F5) | Medium | Low/Medium | Publish atomic state snapshots or guard FSM object access. |
| 8 | Flight-loop EKF timing/stack measurement (F9) | Medium | Low | Add stack high-water and worst-case loop timing measurements with EKF enabled. |
| 9 | Motor output health reporting (F10 plus EasyESC skip status) | Low/Medium | Low/Medium | Track accepted vs commanded outputs and DShot skip/fail status. |
| 10 | Optional parser hardening for iBUS/autonomy and position deltas (F7, F8, F12) | Low | Low | Add parser budgets, strict numeric parsing, and `int64_t` coordinate deltas. |

