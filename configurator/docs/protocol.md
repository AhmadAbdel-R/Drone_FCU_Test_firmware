# FCU Configurator Binary Protocol

Status: Stage 4/5 foundation. The desktop implementation, bench-only USB firmware endpoint, BLE GATT transport, advanced status, profile export, and packaging scripts are wired in. The normal flight image still keeps the endpoint disabled by default.

## Goals

- Keep configurator traffic separate from legacy nRF/CRSF/iBUS/Pi protocols.
- Support USB CDC and BLE GATT using the same binary frames.
- Detect corrupt, partial, stale, and out-of-order traffic.
- Keep firmware parsing bounded and small when the endpoint is added.
- Preserve existing safety behavior by routing commands through existing FCU callbacks.

## Byte Order

All integer fields are little-endian.

## Frame Layout

| Offset | Size | Field | Description |
| --- | ---: | --- | --- |
| 0 | 2 | Sync | ASCII `FC`, bytes `0x46 0x43`. |
| 2 | 1 | Version | Protocol version. Current value is `1`. |
| 3 | 1 | Flags | Bitfield described below. |
| 4 | 2 | Type | Message type. |
| 6 | 2 | Sequence | Request/response sequence number. Wraps at 65535. |
| 8 | 2 | Payload length | Payload byte count. Maximum is 512 bytes in Stage 2. |
| 10 | N | Payload | Message-specific bytes. |
| 10 + N | 4 | CRC-32 | IEEE CRC-32 of every preceding byte in the frame, including sync. |

Minimum frame length is 14 bytes. Maximum Stage 2 frame length is 526 bytes.

## Flags

| Flag | Value | Meaning |
| --- | ---: | --- |
| `ACK_REQUIRED` | `0x01` | Receiver should return ACK or NACK. |
| `RESPONSE` | `0x02` | Frame is a response to a prior request. |
| `ERROR` | `0x04` | Frame reports an error condition. |
| `FRAGMENT` | `0x08` | Payload is a fragment. Fragment reassembly is reserved for a later stage. |
| `FINAL` | `0x10` | Final fragment or final response in a sequence. |

## Core Message Types

| Type | Hex | Direction | Purpose |
| --- | ---: | --- | --- |
| `HELLO` | `0x0001` | App to FCU | Start session, exchange protocol version. |
| `HEARTBEAT` | `0x0002` | Both | Keepalive and disconnect detection. |
| `ACK` | `0x0003` | Both | Acknowledge a request sequence. |
| `NACK` | `0x0004` | Both | Reject a request sequence with a status code. |
| `GET_CAPABILITIES` | `0x0010` | App to FCU | Ask for compiled features and safety gates. |
| `CAPABILITIES` | `0x0011` | FCU to app | Report feature flags, build ID, transports, limits. |
| `GET_STATE` | `0x0020` | App to FCU | Request current high-level FCU state. |
| `STATE` | `0x0021` | FCU to app | Armed, failsafe, mode, battery, loop health, link state. |
| `GET_ADVANCED_STATUS` | `0x0022` | App to FCU | Request task health, servo, notch, and capture summary. |
| `ADVANCED_STATUS` | `0x0023` | FCU to app | Return advanced diagnostic arrays. |
| `GET_SENSOR_STATUS` | `0x0030` | App to FCU | Request sensor presence/health/freshness. |
| `SENSOR_STATUS` | `0x0031` | FCU to app | Per-sensor compiled, detected, healthy, age, value summary. |
| `SENSOR_RESCAN` | `0x0032` | App to FCU | Queue a disarmed I2C/sensor rescan through the safety gate. |
| `GET_CONFIG` | `0x0040` | App to FCU | Read a named config group. |
| `CONFIG` | `0x0041` | FCU to app | Return config group data. |
| `SET_CONFIG` | `0x0042` | App to FCU | Apply runtime config without NVS persistence. |
| `SAVE_CONFIG` | `0x0043` | App to FCU | Persist current config group to NVS. |
| `REVERT_CONFIG` | `0x0044` | App to FCU | Reload config group from NVS. |
| `CALIBRATION_BEGIN` | `0x0050` | App to FCU | Start a calibration flow. |
| `CALIBRATION_PROGRESS` | `0x0051` | FCU to app | Report calibration state/progress/result. |
| `CALIBRATION_COMMIT` | `0x0052` | App to FCU | Commit calibration result when required. |
| `MOTOR_TEST_ARM` | `0x0060` | App to FCU | Enter disarmed motor-test session after safety checks. |
| `MOTOR_TEST_SET` | `0x0061` | App to FCU | Deadman motor command with short timeout. |
| `MOTOR_TEST_STOP` | `0x0062` | App to FCU | Immediate motor-test stop. |
| `MOTOR_TEST_STATUS` | `0x0063` | FCU to app | Echo accepted command, rejection reason, timeout state. |
| `LOG_TEXT` | `0x0070` | FCU to app | Optional framed log line after log separation exists. |
| `CAPTURE_START` | `0x0080` | App to FCU | Start diagnostic capture. |
| `CAPTURE_STOP` | `0x0081` | App to FCU | Stop diagnostic capture. |
| `CAPTURE_CHUNK` | `0x0082` | FCU to app | Return capture bytes or CSV fragments. |
| `TELEMETRY_SUBSCRIBE` | `0x0090` | App to FCU | Request a telemetry group/rate. Current firmware ACKs only. |
| `TELEMETRY_UNSUBSCRIBE` | `0x0091` | App to FCU | Stop a telemetry group. Current firmware ACKs only. |
| `PROFILE_EXPORT` | `0x00A0` | App to FCU, FCU to app | Request/return profile JSON. |
| `PROFILE_IMPORT` | `0x00A1` | App to FCU | Reserved. Current firmware returns `NOT_SUPPORTED`. |
| `FIRMWARE_BEGIN` | `0x00B0` | App to FCU | Reserved firmware update begin. Current firmware returns `NOT_SUPPORTED`. |
| `FIRMWARE_CHUNK` | `0x00B1` | App to FCU | Reserved firmware update chunk. Current firmware returns `NOT_SUPPORTED`. |
| `FIRMWARE_END` | `0x00B2` | App to FCU | Reserved firmware update finish. Current firmware returns `NOT_SUPPORTED`. |
| `SYSTEM_REBOOT` | `0x00C0` | App to FCU | Optional reboot callback. |
| `SYSTEM_BOOTLOADER` | `0x00C1` | App to FCU | Optional bootloader callback. |

## ACK And NACK Payload

ACK and NACK payloads use the same leading fields:

| Offset | Size | Field |
| --- | ---: | --- |
| 0 | 2 | Acknowledged sequence |
| 2 | 2 | Status code |
| 4 | N | Optional UTF-8 detail string |

Common status codes:

| Name | Value |
| --- | ---: |
| `OK` | 0 |
| `BAD_CRC` | 1 |
| `BAD_VERSION` | 2 |
| `BAD_LENGTH` | 3 |
| `UNKNOWN_TYPE` | 4 |
| `BUSY` | 5 |
| `REFUSED` | 6 |
| `UNSAFE_STATE` | 7 |
| `AUTH_REQUIRED` | 8 |
| `TIMEOUT` | 9 |
| `INTERNAL_ERROR` | 10 |
| `INVALID_VALUE` | 11 |
| `NOT_SUPPORTED` | 12 |
| `DEVICE_ERROR` | 13 |

## Sequence Rules

- The app increments sequence numbers per request.
- A response copies the request sequence and sets `RESPONSE`.
- ACK/NACK payloads include the acknowledged sequence so they can still be understood after retransmission or out-of-order delivery.
- Sequence wrap is allowed.
- The firmware endpoint should reject duplicate unsafe commands unless the command is explicitly idempotent.

## Parser Rules

- Discard bytes before sync.
- Reject unsupported protocol versions.
- Reject payloads larger than 512 bytes.
- Wait for more bytes when a frame is incomplete.
- Verify CRC before emitting a frame.
- On CRC failure, discard one byte and resynchronize.
- Do not allocate from payload-provided lengths until the max-length check passes.

## Safety Rules For Future Firmware Endpoint

- The protocol parser runs outside `flightTask`.
- Config writes preserve the current Apply versus Save distinction.
- Logs must not interleave with binary frames on the same stream in release configurator mode.
- Motor testing uses an explicit session: `MOTOR_TEST_ARM`, repeated `MOTOR_TEST_SET`, then `MOTOR_TEST_STOP`. The firmware also stops output on command timeout, session timeout, disconnect-by-timeout, failsafe, throttle, calibration, one-shot test, ESC, or FSM safety changes.
- BLE disconnect falls back to deadman command timeout for motor output. A direct disconnect-stop callback is not enabled yet.

## BLE GATT Transport

BLE transports the same framed bytes as USB:

| Item | UUID |
| --- | --- |
| Service | `b7f3b8f0-6e6a-4d7a-9df7-2c13f0c00001` |
| RX write | `b7f3b8f1-6e6a-4d7a-9df7-2c13f0c00001` |
| TX notify | `b7f3b8f2-6e6a-4d7a-9df7-2c13f0c00001` |
| Info read | `b7f3b8f3-6e6a-4d7a-9df7-2c13f0c00001` |

The Windows app writes frame bytes in 160-byte chunks. Firmware notifies response bytes in 160-byte chunks. The frame parser performs reassembly at the byte-stream level.

## Motor Test Payload

`MOTOR_TEST_SET` payload:

| Offset | Size | Field | Description |
| --- | ---: | --- | --- |
| 0 | 1 | Motor mask | Low nibble selects M1..M4. `0x01` is M1, `0x02` is M2, `0x04` is M3, `0x08` is M4. |
| 1 | 2 | Raw command | DShot raw command, limited by firmware safety policy. Current default max is 600. |
| 3 | 2 | Timeout ms | Per-command output timeout. Current default max is 750 ms. |

## Current Firmware Endpoint

`ENABLE_USB_CONFIG=1` enables the endpoint. The first bench target is `esp32-s3-mini-configurator`.

Implemented request handling:

- `HELLO`
- `HEARTBEAT`
- `GET_CAPABILITIES`
- `GET_STATE`
- `GET_ADVANCED_STATUS`
- `GET_SENSOR_STATUS`
- `SENSOR_RESCAN`
- `GET_CONFIG` for PID config
- `SET_CONFIG` for PID config and mixer bias
- `SAVE_CONFIG` for PID, mixer, and failsafe groups
- `REVERT_CONFIG` for PID
- `CALIBRATION_BEGIN` for gyro, level, mag start, and mag finish
- `MOTOR_TEST_ARM` as a disarmed, idle, safe-state session gate
- `MOTOR_TEST_SET` as a deadman motor output command
- `MOTOR_TEST_STOP` as an immediate session stop
- `TELEMETRY_SUBSCRIBE` and `TELEMETRY_UNSUBSCRIBE` as ACK-only subscriptions
- `PROFILE_EXPORT` as JSON export of PID plus mixer bias
- `PROFILE_IMPORT` as a reserved command returning `NOT_SUPPORTED`
- `FIRMWARE_BEGIN`, `FIRMWARE_CHUNK`, and `FIRMWARE_END` as reserved commands returning `NOT_SUPPORTED`
- `SYSTEM_REBOOT` and `SYSTEM_BOOTLOADER` only if callbacks are registered

Not implemented yet:

- signed firmware update writer
- profile import as a single firmware transaction
- binary telemetry push streams
- hardware-validated BLE stop latency
