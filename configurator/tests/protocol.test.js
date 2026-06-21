import assert from "node:assert/strict";
import test from "node:test";

import {
  DEFAULT_MAX_PAYLOAD_LENGTH,
  FrameFlags,
  MessageType,
  ProtocolParser,
  SequenceCounter,
  StatusCode,
  TelemetryGroup,
  decodeConfigPayload,
  crc32,
  decodeAckPayload,
  encodePidConfig,
  encodeAckFrame,
  encodeAckPayload,
  encodeFrame,
  encodeMotorTestSet,
  encodeTelemetrySubscription,
  fragmentBleFrame,
  motorMaskForMotor,
} from "../src/services/protocol.js";

test("CRC-32 uses the standard IEEE vector", () => {
  assert.equal(crc32(Buffer.from("123456789", "ascii")), 0xcbf43926);
});

test("encodes and decodes one complete frame", () => {
  const payload = Buffer.from([1, 2, 3, 4]);
  const encoded = encodeFrame({
    type: MessageType.GET_STATE,
    seq: 42,
    flags: FrameFlags.ACK_REQUIRED,
    payload,
  });

  const parser = new ProtocolParser();
  const frames = parser.push(encoded);

  assert.equal(frames.length, 1);
  assert.equal(frames[0].version, 1);
  assert.equal(frames[0].type, MessageType.GET_STATE);
  assert.equal(frames[0].seq, 42);
  assert.equal(frames[0].flags, FrameFlags.ACK_REQUIRED);
  assert.deepEqual(frames[0].payload, payload);
  assert.deepEqual(parser.lastErrors, []);
});

test("waits for partial frames", () => {
  const encoded = encodeFrame({
    type: MessageType.HELLO,
    seq: 7,
    payload: Buffer.from("hello", "utf8"),
  });
  const parser = new ProtocolParser();

  assert.deepEqual(parser.push(encoded.subarray(0, 5)), []);
  assert.deepEqual(parser.push(encoded.subarray(5, 11)), []);

  const frames = parser.push(encoded.subarray(11));
  assert.equal(frames.length, 1);
  assert.equal(frames[0].type, MessageType.HELLO);
  assert.equal(frames[0].seq, 7);
});

test("decodes multiple frames from one chunk", () => {
  const first = encodeFrame({ type: MessageType.GET_CAPABILITIES, seq: 1 });
  const second = encodeFrame({ type: MessageType.GET_SENSOR_STATUS, seq: 2 });

  const parser = new ProtocolParser();
  const frames = parser.push(Buffer.concat([first, second]));

  assert.equal(frames.length, 2);
  assert.equal(frames[0].type, MessageType.GET_CAPABILITIES);
  assert.equal(frames[1].type, MessageType.GET_SENSOR_STATUS);
});

test("drops leading noise and resynchronizes", () => {
  const frame = encodeFrame({ type: MessageType.HEARTBEAT, seq: 9 });
  const parser = new ProtocolParser();

  const frames = parser.push(Buffer.concat([Buffer.from([0, 1, 2, 0x46]), frame]));

  assert.equal(frames.length, 1);
  assert.equal(frames[0].type, MessageType.HEARTBEAT);
});

test("reports CRC errors and recovers on the next valid frame", () => {
  const bad = encodeFrame({
    type: MessageType.GET_STATE,
    seq: 10,
    payload: Buffer.from([9, 8, 7]),
  });
  bad[10] ^= 0xff;

  const good = encodeFrame({ type: MessageType.GET_STATE, seq: 11 });
  const parser = new ProtocolParser();
  const frames = parser.push(Buffer.concat([bad, good]));

  assert.equal(frames.length, 1);
  assert.equal(frames[0].seq, 11);
  assert.equal(parser.lastErrors.length, 1);
  assert.equal(parser.lastErrors[0].code, "crc_mismatch");
});

test("rejects unsupported protocol versions", () => {
  const frame = encodeFrame({ type: MessageType.GET_STATE, seq: 12 });
  frame[2] = 2;

  const parser = new ProtocolParser();
  assert.deepEqual(parser.push(frame), []);
  assert.equal(parser.lastErrors.length, 1);
  assert.equal(parser.lastErrors[0].code, "bad_version");
});

test("rejects oversized payloads before waiting for payload bytes", () => {
  const frame = encodeFrame({ type: MessageType.GET_STATE, seq: 13 });
  frame.writeUInt16LE(DEFAULT_MAX_PAYLOAD_LENGTH + 1, 8);

  const parser = new ProtocolParser();
  assert.deepEqual(parser.push(frame.subarray(0, 10)), []);
  assert.equal(parser.lastErrors.length, 1);
  assert.equal(parser.lastErrors[0].code, "payload_too_large");
});

test("encoder enforces payload limit", () => {
  assert.throws(
    () => encodeFrame({
      type: MessageType.SET_CONFIG,
      seq: 14,
      payload: Buffer.alloc(DEFAULT_MAX_PAYLOAD_LENGTH + 1),
    }),
    /exceeds 512/,
  );
});

test("ACK and NACK payload helpers preserve acknowledged sequence and status", () => {
  const ackPayload = encodeAckPayload(55, StatusCode.OK);
  assert.deepEqual(decodeAckPayload(ackPayload), {
    ackedSeq: 55,
    status: StatusCode.OK,
    detail: "",
  });

  const nack = encodeAckFrame({
    seq: 56,
    ackedSeq: 55,
    status: StatusCode.UNSAFE_STATE,
    detail: "disarmed check failed",
  });
  const [frame] = new ProtocolParser().push(nack);
  assert.equal(frame.type, MessageType.NACK);
  assert.equal(frame.flags, FrameFlags.RESPONSE | FrameFlags.ERROR);
  assert.deepEqual(decodeAckPayload(frame.payload), {
    ackedSeq: 55,
    status: StatusCode.UNSAFE_STATE,
    detail: "disarmed check failed",
  });
});

test("sequence counter wraps at 65535", () => {
  const seq = new SequenceCounter(0xfffe);
  assert.equal(seq.next(), 0xfffe);
  assert.equal(seq.next(), 0xffff);
  assert.equal(seq.next(), 0);
  assert.equal(seq.next(), 1);
});

test("PID config payload round-trips signed int16 fields", () => {
  const values = [0, -1, 2, 450, 100, 0, 200, 50, 0, 300, 300, -25];
  const decoded = decodeConfigPayload(Buffer.concat([
    encodePidConfig(values),
    Buffer.from([0, 0, 128, 63, 0]),
  ]));

  assert.deepEqual(decoded.pid, values);
  assert.equal(decoded.mixBias, 1);
});

test("motor test set payload encodes mask, raw, and timeout", () => {
  const payload = encodeMotorTestSet({
    motorMask: motorMaskForMotor(3),
    raw: 300,
    timeoutMs: 350,
  });

  assert.equal(payload.length, 5);
  assert.equal(payload[0], 0x04);
  assert.equal(payload.readUInt16LE(1), 300);
  assert.equal(payload.readUInt16LE(3), 350);
});

test("telemetry subscription payload encodes group and rate", () => {
  const payload = encodeTelemetrySubscription(TelemetryGroup.FILTERS, 25);
  assert.equal(payload.length, 3);
  assert.equal(payload[0], TelemetryGroup.FILTERS);
  assert.equal(payload.readUInt16LE(1), 25);
});

test("BLE frame chunking preserves byte order", () => {
  const frame = encodeFrame({
    type: MessageType.PROFILE_IMPORT,
    seq: 77,
    payload: Buffer.alloc(350, 0x5a),
  });
  const chunks = fragmentBleFrame(frame, 160);
  assert.equal(chunks.length, 3);
  assert.equal(Math.max(...chunks.map((chunk) => chunk.length)) <= 160, true);
  assert.deepEqual(Buffer.concat(chunks), frame);
});
