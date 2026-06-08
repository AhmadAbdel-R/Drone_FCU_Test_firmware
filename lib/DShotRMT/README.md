# Local DShotRMT Fork

This is the local DShot/RMT driver used by the FCU firmware. It started from
the Arduino `DShotRMT` library, but this repository keeps a patched copy because
the flight firmware depends on specific ESP32-S3 and bidirectional DShot
behavior.

## What This Library Does Here

- Generates DShot150/300/600/1200 frames using the ESP-IDF 5 RMT TX API.
- Uses DShot300 as the FCU's safe default.
- Supports optional bidirectional DShot eRPM receive.
- Keeps TX payload and RX symbol storage persistent, not stack-backed.
- Starts the RX window from the RMT TX-done callback.
- Defers eRPM decode out of the ISR.
- Tracks telemetry stats for no-edge, decode, CRC, period, overrun, good, and
  bad frames.

## Flight-Loop Constraints

Any future changes must preserve these constraints:

- No heap allocation in the normal flight-loop send/telemetry path.
- No RMT channel attach/detach/recreate during normal motor output.
- Normal DShot output must remain reliable if telemetry fails.
- Bad BDShot frames must never publish a new RPM value.
- DShot300 remains the default unless bench data proves another speed is safer.

## Bidirectional DShot Notes

BDShot is compiled off by default in the flight firmware:

```ini
-D FCU_DSHOT_BIDIR=1
```

Each motor signal line needs a pull-up to 3.3 V at the FC end before enabling
BDShot. Validate with props off.

The eRPM frame path decodes the 21-bit differential GCR response into a
compressed eRPM period, validates the checksum, expands the period, and returns
electrical RPM as `uint32_t`.

## Public Calls Used By The Firmware

| API | Use |
|---|---|
| `begin()` | Create persistent RMT TX/RX channels and encoder |
| `sendThrottle(raw)` | Queue one DShot throttle frame |
| `setMotorSpinDirection(reversed)` | ESC direction command during init |
| `getTelemetry()` | Poll decoded eRPM/full telemetry result |
| `getTelemetryStats()` | Read telemetry counters |
| `setTxBufferSymbols(symbols)` | Tune RMT TX memory sizing before `begin()` |

## Integration Path

`EasyEscMotor` in `lib/easy-esc-esp32` owns one `DShotRMT` instance per motor.
The EasyESC facade polls `getTelemetry()` before sending the next throttle frame
and exposes cached eRPM through `EasyEscMotor::telemetry()`. The RPM filter in
`src/RpmNotchFilter.cpp` consumes that cached sample.

## Upstream

Original project: https://github.com/derdoktor667/DShotRMT

Keep upstream license notices intact when syncing or rebasing this local copy.
