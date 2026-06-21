import {
  CalibrationId,
  ConfigGroup,
  FrameFlags,
  MessageType,
  ProtocolParser,
  SequenceCounter,
  StatusCode,
  TelemetryGroup,
  decodeAckPayload,
  decodeConfigPayload,
  decodeJsonPayload,
  encodeFirmwareBegin,
  encodeFirmwareChunk,
  encodeFrame,
  encodeJsonPayload,
  encodeMixConfig,
  encodeMotorTestSet,
  encodePidConfig,
  encodeTelemetrySubscription,
  motorMaskForMotor,
} from "./protocol.js";

export class FcuRequestError extends Error {
  constructor(message, status = StatusCode.INTERNAL_ERROR) {
    super(message);
    this.name = "FcuRequestError";
    this.status = status;
  }
}

export class FcuClient {
  constructor(transport, { timeoutMs = 1200 } = {}) {
    this.transport = transport;
    this.timeoutMs = timeoutMs;
    this.parser = new ProtocolParser();
    this.seq = new SequenceCounter(1);
    this.pending = new Map();
    this.requestQueue = Promise.resolve();
    this.unsubData = null;
    this.stats = {
      rxFrames: 0,
      txFrames: 0,
      timeouts: 0,
      sequenceGaps: 0,
      crcErrors: 0,
      reconnects: 0,
    };
  }

  async connect(options) {
    this.parser.reset();
    this.unsubData?.();
    this.unsubData = this.transport.onData((chunk) => this.handleData(chunk));
    await this.transport.connect(options);
    return this.hello();
  }

  async disconnect() {
    for (const pending of this.pending.values()) {
      clearTimeout(pending.timer);
      pending.reject(new FcuRequestError("Disconnected", StatusCode.TIMEOUT));
    }
    this.pending.clear();
    this.parser.reset();
    await this.transport.disconnect();
  }

  isConnected() {
    return this.transport.isConnected();
  }

  async request(type, payload = new Uint8Array(0), options = {}) {
    const queued = this.requestQueue.then(
      () => this.runRequest(type, payload, options),
      () => this.runRequest(type, payload, options),
    );
    this.requestQueue = queued.catch(() => {});
    return queued;
  }

  async runRequest(type, payload = new Uint8Array(0), { timeoutMs = this.timeoutMs } = {}) {
    const seq = this.seq.next();
    const frame = encodeFrame({
      type,
      seq,
      flags: FrameFlags.ACK_REQUIRED,
      payload,
    });
    const promise = new Promise((resolve, reject) => {
      const timer = setTimeout(() => {
        this.pending.delete(seq);
        this.stats.timeouts += 1;
        reject(new FcuRequestError(`Request 0x${type.toString(16)} timed out`, StatusCode.TIMEOUT));
      }, timeoutMs);
      this.pending.set(seq, { resolve, reject, timer, type });
    });
    await this.transport.write(frame);
    this.stats.txFrames += 1;
    return promise;
  }

  handleData(chunk) {
    const frames = this.parser.push(chunk);
    for (const error of this.parser.lastErrors) {
      if (error.code === "crc_mismatch") {
        this.stats.crcErrors += 1;
      }
    }
    for (const frame of frames) {
      this.stats.rxFrames += 1;
      const pending = this.pending.get(frame.seq);
      if (!pending) {
        continue;
      }
      clearTimeout(pending.timer);
      this.pending.delete(frame.seq);
      if (frame.type === MessageType.NACK) {
        const nack = decodeAckPayload(frame.payload);
        pending.reject(new FcuRequestError(nack.detail || "Request rejected", nack.status));
      } else if (frame.type === MessageType.ACK) {
        pending.resolve({ frame, ack: decodeAckPayload(frame.payload) });
      } else {
        pending.resolve({ frame });
      }
    }
  }

  async hello() {
    const { frame } = await this.request(MessageType.HELLO);
    return decodeJsonPayload(frame.payload);
  }

  async getCapabilities() {
    const { frame } = await this.request(MessageType.GET_CAPABILITIES);
    return decodeJsonPayload(frame.payload);
  }

  async getState() {
    const { frame } = await this.request(MessageType.GET_STATE);
    return decodeJsonPayload(frame.payload);
  }

  async getAdvancedStatus() {
    const { frame } = await this.request(MessageType.GET_ADVANCED_STATUS);
    return decodeJsonPayload(frame.payload);
  }

  async getSensorStatus() {
    const { frame } = await this.request(MessageType.GET_SENSOR_STATUS);
    return decodeJsonPayload(frame.payload);
  }

  async getConfig() {
    const { frame } = await this.request(MessageType.GET_CONFIG, Uint8Array.from([ConfigGroup.PID]));
    return decodeConfigPayload(frame.payload);
  }

  async applyPid(values) {
    return this.request(MessageType.SET_CONFIG, encodePidConfig(values));
  }

  async applyMixBias(value) {
    return this.request(MessageType.SET_CONFIG, encodeMixConfig(value));
  }

  async savePid() {
    return this.request(MessageType.SAVE_CONFIG, Uint8Array.from([ConfigGroup.PID]));
  }

  async reloadPid() {
    return this.request(MessageType.REVERT_CONFIG, Uint8Array.from([ConfigGroup.PID]));
  }

  async calibrate(id) {
    return this.request(MessageType.CALIBRATION_BEGIN, Uint8Array.from([id]));
  }

  async calibrateGyro() {
    return this.calibrate(CalibrationId.GYRO);
  }

  async calibrateLevel() {
    return this.calibrate(CalibrationId.LEVEL);
  }

  async rescanSensors() {
    return this.request(MessageType.SENSOR_RESCAN);
  }

  async subscribeTelemetry(group = TelemetryGroup.DASHBOARD, rateHz = 5) {
    return this.request(MessageType.TELEMETRY_SUBSCRIBE, encodeTelemetrySubscription(group, rateHz));
  }

  async unsubscribeTelemetry(group = TelemetryGroup.NONE) {
    return this.request(MessageType.TELEMETRY_UNSUBSCRIBE, Uint8Array.from([group]));
  }

  async exportProfile() {
    const { frame } = await this.request(MessageType.PROFILE_EXPORT);
    return decodeJsonPayload(frame.payload);
  }

  async importProfile(profile) {
    return this.request(MessageType.PROFILE_IMPORT, encodeJsonPayload(profile));
  }

  async beginFirmwareUpdate(metadata) {
    return this.request(MessageType.FIRMWARE_BEGIN, encodeFirmwareBegin(metadata), { timeoutMs: 5000 });
  }

  async sendFirmwareChunk(offset, chunk) {
    return this.request(MessageType.FIRMWARE_CHUNK, encodeFirmwareChunk(offset, chunk), { timeoutMs: 5000 });
  }

  async finishFirmwareUpdate() {
    return this.request(MessageType.FIRMWARE_END, new Uint8Array(0), { timeoutMs: 5000 });
  }

  async reboot() {
    return this.request(MessageType.SYSTEM_REBOOT);
  }

  async enterBootloader() {
    return this.request(MessageType.SYSTEM_BOOTLOADER);
  }

  async armMotorTest() {
    return this.request(MessageType.MOTOR_TEST_ARM);
  }

  async setMotorTest(motor, raw, timeoutMs) {
    return this.request(MessageType.MOTOR_TEST_SET, encodeMotorTestSet({
      motorMask: motorMaskForMotor(motor),
      raw,
      timeoutMs,
    }));
  }

  async stopMotorTest() {
    return this.request(MessageType.MOTOR_TEST_STOP);
  }
}
