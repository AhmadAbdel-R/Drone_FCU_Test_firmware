import { Transport } from "./transport.js";
import {
  ConfigGroup,
  FrameFlags,
  MessageType,
  ProtocolParser,
  StatusCode,
  decodeConfigPayload,
  encodeAckFrame,
  encodeFrame,
  encodeJsonPayload,
} from "./protocol.js";

function pidConfigPayload(pid, mixBias) {
  const payload = Buffer.alloc(30);
  payload[0] = ConfigGroup.PID;
  pid.forEach((value, index) => payload.writeInt16LE(value, 1 + index * 2));
  payload.writeFloatLE(mixBias, 25);
  payload[29] = 0;
  return payload;
}

export class MockTransport extends Transport {
  constructor() {
    super("Mock FCU");
    this.parser = new ProtocolParser();
    this.startedAt = Date.now();
    this.pid = [0, 0, 0, 450, 100, 0, 200, 50, 0, 300, 300, 0];
    this.mixBias = 1.0;
    this.motorSessionArmed = false;
    this.motorSetCount = 0;
    this.motorStopCount = 0;
    this.lastMotorCommand = null;
    this.faultInjection = {
      failsafe: false,
      gpsFix: true,
      tofMissing: false,
    };
  }

  async discover() {
    return [{
      path: "MOCK",
      friendlyName: "Simulated ESP32-S3 Mini FCU",
      manufacturer: "AeroForge Mock",
      serialNumber: "MOCK-001",
      transport: "mock",
    }];
  }

  async connect() {
    this.setConnected(true, { path: "MOCK" });
  }

  async disconnect() {
    this.setConnected(false, { path: "MOCK" });
    this.motorSessionArmed = false;
    this.lastMotorCommand = null;
    this.parser.reset();
  }

  async write(data) {
    if (!this.connected) {
      throw new Error("Mock FCU is not connected");
    }
    const frames = this.parser.push(data);
    for (const frame of frames) {
      queueMicrotask(() => this.respond(frame));
    }
  }

  respond(frame) {
    const send = (type, payload, flags = FrameFlags.RESPONSE | FrameFlags.FINAL) => {
      this.emitData(encodeFrame({ type, seq: frame.seq, flags, payload }));
    };
    const ack = (status = StatusCode.OK, detail = "") => {
      this.emitData(encodeAckFrame({ seq: frame.seq, ackedSeq: frame.seq, status, detail }));
    };

    switch (frame.type) {
      case MessageType.HELLO:
        send(MessageType.HELLO, Buffer.from(JSON.stringify({
          app: "AeroForge Configurator",
          proto: 1,
          target: "esp32-s3-mini",
          transport: "mock",
          build: "mock-fcu",
          ble: false,
        })));
        break;
      case MessageType.GET_CAPABILITIES:
        send(MessageType.CAPABILITIES, Buffer.from(JSON.stringify({
          proto: 1,
          maxPayload: 512,
          usb: true,
          ble: false,
          pid: true,
          sensors: true,
          motorTest: "deadman",
          nvs: true,
          advancedStatus: true,
          profiles: true,
          sensorRescan: true,
          firmwareUpdate: false,
          telemetrySubscribe: "ack-only",
          logsOnUsb: false,
          transportBle: false,
        })));
        break;
      case MessageType.GET_STATE:
        send(MessageType.STATE, Buffer.from(JSON.stringify(this.statePayload())));
        break;
      case MessageType.GET_ADVANCED_STATUS:
        send(MessageType.ADVANCED_STATUS, Buffer.from(JSON.stringify(this.advancedPayload())));
        break;
      case MessageType.GET_SENSOR_STATUS:
        send(MessageType.SENSOR_STATUS, Buffer.from(JSON.stringify(this.sensorPayload())));
        break;
      case MessageType.SENSOR_RESCAN:
        ack();
        break;
      case MessageType.GET_CONFIG:
        send(MessageType.CONFIG, pidConfigPayload(this.pid, this.mixBias));
        break;
      case MessageType.SET_CONFIG: {
        if (frame.payload[0] === ConfigGroup.PID && frame.payload.length >= 25) {
          const next = [];
          for (let i = 0; i < 12; i += 1) {
            next.push(frame.payload.readInt16LE(1 + i * 2));
          }
          this.pid = next;
          ack();
        } else if (frame.payload[0] === ConfigGroup.MIX && frame.payload.length >= 5) {
          this.mixBias = frame.payload.readFloatLE(1);
          ack();
        } else {
          ack(StatusCode.NOT_SUPPORTED, "config group");
        }
        break;
      }
      case MessageType.SAVE_CONFIG:
      case MessageType.REVERT_CONFIG:
      case MessageType.CALIBRATION_BEGIN:
        ack();
        break;
      case MessageType.MOTOR_TEST_ARM:
        this.motorSessionArmed = true;
        this.lastMotorCommand = null;
        ack();
        break;
      case MessageType.MOTOR_TEST_SET: {
        if (!this.motorSessionArmed) {
          ack(StatusCode.UNSAFE_STATE, "motor session not armed");
          break;
        }
        if (frame.payload.length < 5) {
          ack(StatusCode.BAD_LENGTH, "motor set payload");
          break;
        }
        const motorMask = frame.payload[0];
        const raw = frame.payload.readUInt16LE(1);
        const timeoutMs = frame.payload.readUInt16LE(3);
        if ((motorMask & 0x0f) === 0 || (motorMask & 0xf0) !== 0 || timeoutMs === 0) {
          ack(StatusCode.INVALID_VALUE, "motor set value");
          break;
        }
        this.motorSetCount += 1;
        this.lastMotorCommand = { motorMask, raw, timeoutMs };
        ack();
        break;
      }
      case MessageType.MOTOR_TEST_STOP:
        this.motorSessionArmed = false;
        this.lastMotorCommand = null;
        this.motorStopCount += 1;
        ack();
        break;
      case MessageType.TELEMETRY_SUBSCRIBE:
      case MessageType.TELEMETRY_UNSUBSCRIBE:
        ack();
        break;
      case MessageType.PROFILE_EXPORT:
        send(MessageType.PROFILE_EXPORT, encodeJsonPayload(this.profilePayload()));
        break;
      case MessageType.PROFILE_IMPORT:
        ack(StatusCode.NOT_SUPPORTED, "profile import uses validated config writes");
        break;
      case MessageType.FIRMWARE_BEGIN:
      case MessageType.FIRMWARE_CHUNK:
      case MessageType.FIRMWARE_END:
        ack(StatusCode.NOT_SUPPORTED, "firmware update transport not enabled");
        break;
      case MessageType.SYSTEM_REBOOT:
      case MessageType.SYSTEM_BOOTLOADER:
        ack(StatusCode.NOT_SUPPORTED, "system action disabled in mock");
        break;
      default:
        ack(StatusCode.UNKNOWN_TYPE, "unknown");
        break;
    }
  }

  statePayload() {
    const t = (Date.now() - this.startedAt) / 1000;
    const roll = Math.sin(t * 0.7) * 3;
    const pitch = Math.cos(t * 0.5) * 2;
    return {
      up: Math.floor(Date.now() - this.startedAt),
      arm: 0,
      fs: this.faultInjection.failsafe ? 1 : 0,
      reason: this.faultInjection.failsafe ? 1 : 0,
      thr: 0,
      bat: 15.72,
      pct: 84,
      loop: 500,
      heap: 184320,
      att: [roll, pitch, (t * 12) % 360],
      mot: [0, 0, 0, 0],
      rc: [1, 99, -42, 150],
      gps: [this.faultInjection.gpsFix ? 1 : 0, this.faultInjection.gpsFix ? 12 : 0],
      tof: this.faultInjection.tofMissing ? 0 : 812,
    };
  }

  sensorPayload() {
    return {
      imu: [1, 1, 1],
      baro: [1, 1, 1, 22],
      tof: [1, this.faultInjection.tofMissing ? 0 : 1, this.faultInjection.tofMissing ? 0 : 1, 30, 812],
      gps: [1, 1, this.faultInjection.gpsFix ? 1 : 0, 180, this.faultInjection.gpsFix ? 12 : 0],
      mag: [1, 1, 1, 42.5],
      batt: [1, 15.72, 84],
      pi: [0, 0, 4294967295],
      servo: [1, 1500, 1500],
      age: 0,
    };
  }

  advancedPayload() {
    const t = (Date.now() - this.startedAt) / 1000;
    return {
      health: [184320, 160512, 0, 0, 1, 742, Math.floor(Date.now() - this.startedAt)],
      servo: [1, 1500 + Math.round(Math.sin(t) * 80), 1500, 1500, 1500],
      notch: [1, 1, 142.5, 80, 260, 5.0, 0, 1, 145.0, 0.82],
      capture: [0, 0, 1, 0, 1024, 2048, 1000.0],
    };
  }

  profilePayload() {
    return {
      schema: "aeroforge.profile.v1",
      target: "esp32-s3-mini",
      protocol: 1,
      pid: this.pid.slice(),
      mixBias: this.mixBias,
    };
  }
}
