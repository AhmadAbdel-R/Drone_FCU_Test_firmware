export const PROTOCOL_VERSION = 1;
export const SYNC_A = 0x46; // F
export const SYNC_B = 0x43; // C
export const HEADER_LENGTH = 10;
export const CRC_LENGTH = 4;
export const MIN_FRAME_LENGTH = HEADER_LENGTH + CRC_LENGTH;
export const DEFAULT_MAX_PAYLOAD_LENGTH = 512;
export const DEFAULT_BLE_CHUNK_LENGTH = 160;

export const BleGatt = Object.freeze({
  SERVICE_UUID: "b7f3b8f0-6e6a-4d7a-9df7-2c13f0c00001",
  RX_UUID: "b7f3b8f1-6e6a-4d7a-9df7-2c13f0c00001",
  TX_UUID: "b7f3b8f2-6e6a-4d7a-9df7-2c13f0c00001",
  INFO_UUID: "b7f3b8f3-6e6a-4d7a-9df7-2c13f0c00001",
});

export const FrameFlags = Object.freeze({
  ACK_REQUIRED: 0x01,
  RESPONSE: 0x02,
  ERROR: 0x04,
  FRAGMENT: 0x08,
  FINAL: 0x10,
});

export const MessageType = Object.freeze({
  HELLO: 0x0001,
  HEARTBEAT: 0x0002,
  ACK: 0x0003,
  NACK: 0x0004,
  GET_CAPABILITIES: 0x0010,
  CAPABILITIES: 0x0011,
  GET_STATE: 0x0020,
  STATE: 0x0021,
  GET_ADVANCED_STATUS: 0x0022,
  ADVANCED_STATUS: 0x0023,
  GET_SENSOR_STATUS: 0x0030,
  SENSOR_STATUS: 0x0031,
  SENSOR_RESCAN: 0x0032,
  GET_CONFIG: 0x0040,
  CONFIG: 0x0041,
  SET_CONFIG: 0x0042,
  SAVE_CONFIG: 0x0043,
  REVERT_CONFIG: 0x0044,
  CALIBRATION_BEGIN: 0x0050,
  CALIBRATION_PROGRESS: 0x0051,
  CALIBRATION_COMMIT: 0x0052,
  MOTOR_TEST_ARM: 0x0060,
  MOTOR_TEST_SET: 0x0061,
  MOTOR_TEST_STOP: 0x0062,
  MOTOR_TEST_STATUS: 0x0063,
  LOG_TEXT: 0x0070,
  CAPTURE_START: 0x0080,
  CAPTURE_STOP: 0x0081,
  CAPTURE_CHUNK: 0x0082,
  TELEMETRY_SUBSCRIBE: 0x0090,
  TELEMETRY_UNSUBSCRIBE: 0x0091,
  PROFILE_EXPORT: 0x00a0,
  PROFILE_IMPORT: 0x00a1,
  FIRMWARE_BEGIN: 0x00b0,
  FIRMWARE_CHUNK: 0x00b1,
  FIRMWARE_END: 0x00b2,
  SYSTEM_REBOOT: 0x00c0,
  SYSTEM_BOOTLOADER: 0x00c1,
});

export const StatusCode = Object.freeze({
  OK: 0,
  BAD_CRC: 1,
  BAD_VERSION: 2,
  BAD_LENGTH: 3,
  UNKNOWN_TYPE: 4,
  BUSY: 5,
  REFUSED: 6,
  UNSAFE_STATE: 7,
  AUTH_REQUIRED: 8,
  TIMEOUT: 9,
  INTERNAL_ERROR: 10,
  INVALID_VALUE: 11,
  NOT_SUPPORTED: 12,
  DEVICE_ERROR: 13,
});

export const ConfigGroup = Object.freeze({
  PID: 1,
  MIX: 2,
  FAILSAFE: 3,
});

export const CalibrationId = Object.freeze({
  GYRO: 1,
  LEVEL: 2,
  MAG_START: 3,
  MAG_FINISH: 4,
});

export const MotorTestLimits = Object.freeze({
  DEFAULT_RAW: 300,
  MAX_RAW: 600,
  DEFAULT_TIMEOUT_MS: 350,
  MAX_TIMEOUT_MS: 750,
});

export const TelemetryGroup = Object.freeze({
  NONE: 0,
  DASHBOARD: 1,
  RECEIVER: 2,
  SENSORS: 3,
  MOTORS: 4,
  FILTERS: 5,
  GPS: 6,
  SYSTEM: 7,
});

let crcTable = null;

function buildCrcTable() {
  const table = new Uint32Array(256);
  for (let i = 0; i < 256; i += 1) {
    let c = i;
    for (let bit = 0; bit < 8; bit += 1) {
      c = (c & 1) ? (0xedb88320 ^ (c >>> 1)) : (c >>> 1);
    }
    table[i] = c >>> 0;
  }
  return table;
}

function asUint8Array(value) {
  if (value == null) {
    return new Uint8Array(0);
  }
  if (value instanceof Uint8Array) {
    return value;
  }
  if (value instanceof ArrayBuffer) {
    return new Uint8Array(value);
  }
  if (ArrayBuffer.isView(value)) {
    return new Uint8Array(value.buffer, value.byteOffset, value.byteLength);
  }
  if (Array.isArray(value)) {
    return Uint8Array.from(value);
  }
  throw new TypeError("Expected payload/chunk to be a Uint8Array, ArrayBuffer, or byte array");
}

function assertUint(name, value, max) {
  if (!Number.isInteger(value) || value < 0 || value > max) {
    throw new RangeError(`${name} must be an integer from 0 to ${max}`);
  }
}

export function crc32(input) {
  const bytes = asUint8Array(input);
  if (crcTable == null) {
    crcTable = buildCrcTable();
  }

  let crc = 0xffffffff;
  for (const byte of bytes) {
    crc = crcTable[(crc ^ byte) & 0xff] ^ (crc >>> 8);
  }
  return (crc ^ 0xffffffff) >>> 0;
}

export function encodeFrame({
  type,
  seq,
  flags = 0,
  payload = new Uint8Array(0),
  version = PROTOCOL_VERSION,
  maxPayloadLength = DEFAULT_MAX_PAYLOAD_LENGTH,
}) {
  assertUint("type", type, 0xffff);
  assertUint("seq", seq, 0xffff);
  assertUint("flags", flags, 0xff);
  assertUint("version", version, 0xff);

  const payloadBytes = asUint8Array(payload);
  if (payloadBytes.byteLength > maxPayloadLength) {
    throw new RangeError(`payload length ${payloadBytes.byteLength} exceeds ${maxPayloadLength}`);
  }

  const frame = Buffer.alloc(HEADER_LENGTH + payloadBytes.byteLength + CRC_LENGTH);
  frame[0] = SYNC_A;
  frame[1] = SYNC_B;
  frame[2] = version;
  frame[3] = flags;
  frame.writeUInt16LE(type, 4);
  frame.writeUInt16LE(seq, 6);
  frame.writeUInt16LE(payloadBytes.byteLength, 8);
  Buffer.from(payloadBytes).copy(frame, HEADER_LENGTH);

  const crc = crc32(frame.subarray(0, frame.length - CRC_LENGTH));
  frame.writeUInt32LE(crc, frame.length - CRC_LENGTH);
  return frame;
}

export function encodeAckPayload(ackedSeq, status = StatusCode.OK, detail = "") {
  assertUint("ackedSeq", ackedSeq, 0xffff);
  assertUint("status", status, 0xffff);

  const detailBytes = Buffer.from(String(detail), "utf8");
  const payload = Buffer.alloc(4 + detailBytes.length);
  payload.writeUInt16LE(ackedSeq, 0);
  payload.writeUInt16LE(status, 2);
  detailBytes.copy(payload, 4);
  return payload;
}

export function decodeAckPayload(payload) {
  const bytes = Buffer.from(asUint8Array(payload));
  if (bytes.length < 4) {
    throw new RangeError("ACK/NACK payload must be at least 4 bytes");
  }
  return {
    ackedSeq: bytes.readUInt16LE(0),
    status: bytes.readUInt16LE(2),
    detail: bytes.subarray(4).toString("utf8"),
  };
}

export function encodeAckFrame({ seq, ackedSeq, status = StatusCode.OK, detail = "" }) {
  return encodeFrame({
    type: status === StatusCode.OK ? MessageType.ACK : MessageType.NACK,
    seq,
    flags: FrameFlags.RESPONSE | (status === StatusCode.OK ? 0 : FrameFlags.ERROR),
    payload: encodeAckPayload(ackedSeq, status, detail),
  });
}

export class ProtocolParseError extends Error {
  constructor(code, message) {
    super(message);
    this.name = "ProtocolParseError";
    this.code = code;
  }
}

function findSync(buffer) {
  for (let i = 0; i + 1 < buffer.length; i += 1) {
    if (buffer[i] === SYNC_A && buffer[i + 1] === SYNC_B) {
      return i;
    }
  }
  return -1;
}

export class ProtocolParser {
  constructor({ maxPayloadLength = DEFAULT_MAX_PAYLOAD_LENGTH } = {}) {
    assertUint("maxPayloadLength", maxPayloadLength, 0xffff);
    this.maxPayloadLength = maxPayloadLength;
    this.buffer = Buffer.alloc(0);
    this.lastErrors = [];
  }

  push(chunk) {
    const chunkBytes = Buffer.from(asUint8Array(chunk));
    this.lastErrors = [];
    if (chunkBytes.length > 0) {
      this.buffer = Buffer.concat([this.buffer, chunkBytes]);
    }

    const frames = [];
    while (this.buffer.length > 0) {
      const syncIndex = findSync(this.buffer);
      if (syncIndex < 0) {
        this.buffer = this.buffer[this.buffer.length - 1] === SYNC_A
          ? this.buffer.subarray(this.buffer.length - 1)
          : Buffer.alloc(0);
        break;
      }

      if (syncIndex > 0) {
        this.buffer = this.buffer.subarray(syncIndex);
      }

      if (this.buffer.length < HEADER_LENGTH) {
        break;
      }

      const version = this.buffer[2];
      if (version !== PROTOCOL_VERSION) {
        this.lastErrors.push(new ProtocolParseError("bad_version", `unsupported protocol version ${version}`));
        this.buffer = this.buffer.subarray(1);
        continue;
      }

      const flags = this.buffer[3];
      const type = this.buffer.readUInt16LE(4);
      const seq = this.buffer.readUInt16LE(6);
      const payloadLength = this.buffer.readUInt16LE(8);

      if (payloadLength > this.maxPayloadLength) {
        this.lastErrors.push(new ProtocolParseError("payload_too_large", `payload length ${payloadLength} exceeds ${this.maxPayloadLength}`));
        this.buffer = this.buffer.subarray(1);
        continue;
      }

      const totalLength = HEADER_LENGTH + payloadLength + CRC_LENGTH;
      if (this.buffer.length < totalLength) {
        break;
      }

      const frameBytes = this.buffer.subarray(0, totalLength);
      const expectedCrc = frameBytes.readUInt32LE(totalLength - CRC_LENGTH);
      const actualCrc = crc32(frameBytes.subarray(0, totalLength - CRC_LENGTH));
      if (expectedCrc !== actualCrc) {
        this.lastErrors.push(new ProtocolParseError("crc_mismatch", "frame CRC mismatch"));
        this.buffer = this.buffer.subarray(1);
        continue;
      }

      frames.push({
        version,
        flags,
        type,
        seq,
        payload: Buffer.from(frameBytes.subarray(HEADER_LENGTH, HEADER_LENGTH + payloadLength)),
      });
      this.buffer = this.buffer.subarray(totalLength);
    }

    return frames;
  }

  reset() {
    this.buffer = Buffer.alloc(0);
    this.lastErrors = [];
  }
}

export class SequenceCounter {
  constructor(start = 1) {
    assertUint("start", start, 0xffff);
    this.value = start;
  }

  next() {
    const seq = this.value;
    this.value = (this.value + 1) & 0xffff;
    return seq;
  }
}

export function encodeJsonPayload(value) {
  return Buffer.from(JSON.stringify(value), "utf8");
}

export function decodeJsonPayload(payload) {
  const text = Buffer.from(asUint8Array(payload)).toString("utf8");
  return text.length === 0 ? null : JSON.parse(text);
}

export function encodePidConfig(values) {
  if (!Array.isArray(values) || values.length !== 12) {
    throw new RangeError("PID config requires exactly 12 values");
  }
  const payload = Buffer.alloc(1 + values.length * 2);
  payload[0] = ConfigGroup.PID;
  values.forEach((value, index) => {
    if (!Number.isInteger(value) || value < -32768 || value > 32767) {
      throw new RangeError(`PID field ${index} is outside int16 range`);
    }
    payload.writeInt16LE(value, 1 + index * 2);
  });
  return payload;
}

export function decodeConfigPayload(payload) {
  const bytes = Buffer.from(asUint8Array(payload));
  if (bytes.length < 1) {
    throw new RangeError("config payload is empty");
  }
  const group = bytes[0];
  if (group !== ConfigGroup.PID) {
    return { group, raw: bytes };
  }
  if (bytes.length < 30) {
    throw new RangeError("PID config payload is too short");
  }
  const pid = [];
  for (let i = 0; i < 12; i += 1) {
    pid.push(bytes.readInt16LE(1 + i * 2));
  }
  return {
    group,
    pid,
    mixBias: bytes.readFloatLE(25),
    flags: bytes[29],
  };
}

export function encodeMixConfig(mixBias) {
  if (!Number.isFinite(mixBias)) {
    throw new RangeError("mix bias must be finite");
  }
  const payload = Buffer.alloc(5);
  payload[0] = ConfigGroup.MIX;
  payload.writeFloatLE(mixBias, 1);
  return payload;
}

export function motorMaskForMotor(motor) {
  assertUint("motor", motor, 4);
  if (motor < 1) {
    throw new RangeError("motor must be from 1 to 4");
  }
  return 1 << (motor - 1);
}

export function encodeMotorTestSet({
  motorMask,
  raw = MotorTestLimits.DEFAULT_RAW,
  timeoutMs = MotorTestLimits.DEFAULT_TIMEOUT_MS,
}) {
  assertUint("motorMask", motorMask, 0xff);
  assertUint("raw", raw, MotorTestLimits.MAX_RAW);
  assertUint("timeoutMs", timeoutMs, MotorTestLimits.MAX_TIMEOUT_MS);
  if ((motorMask & 0x0f) === 0 || (motorMask & 0xf0) !== 0) {
    throw new RangeError("motorMask must select one or more motors from M1 to M4");
  }
  if (timeoutMs === 0) {
    throw new RangeError("timeoutMs must be greater than zero");
  }

  const payload = Buffer.alloc(5);
  payload[0] = motorMask;
  payload.writeUInt16LE(raw, 1);
  payload.writeUInt16LE(timeoutMs, 3);
  return payload;
}

export function encodeTelemetrySubscription(group, rateHz = 5) {
  assertUint("group", group, 0xff);
  assertUint("rateHz", rateHz, 0xffff);
  const payload = Buffer.alloc(3);
  payload[0] = group;
  payload.writeUInt16LE(rateHz, 1);
  return payload;
}

export function encodeFirmwareBegin(metadata) {
  return encodeJsonPayload(metadata ?? {});
}

export function encodeFirmwareChunk(offset, chunk) {
  assertUint("offset", offset, 0xffffffff);
  const chunkBytes = asUint8Array(chunk);
  const payload = Buffer.alloc(4 + chunkBytes.byteLength);
  payload.writeUInt32LE(offset, 0);
  Buffer.from(chunkBytes).copy(payload, 4);
  return payload;
}

export function fragmentBleFrame(frame, chunkLength = DEFAULT_BLE_CHUNK_LENGTH) {
  assertUint("chunkLength", chunkLength, 0xffff);
  if (chunkLength === 0) {
    throw new RangeError("chunkLength must be greater than zero");
  }
  const bytes = Buffer.from(asUint8Array(frame));
  const chunks = [];
  for (let offset = 0; offset < bytes.length; offset += chunkLength) {
    chunks.push(bytes.subarray(offset, offset + chunkLength));
  }
  return chunks;
}
