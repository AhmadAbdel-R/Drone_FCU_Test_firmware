<script setup>
import { computed, onBeforeUnmount, onMounted, ref, watch } from "vue";

import AttitudeScene from "./components/AttitudeScene.vue";
import { appIdentity } from "./config/appIdentity.js";
import { BleTransport } from "./services/bleTransport.js";
import { FcuClient } from "./services/fcuClient.js";
import { prepareFirmwareFile } from "./services/firmwareUpdater.js";
import { MockTransport } from "./services/mockTransport.js";
import {
  createProfileFromConfig,
  formatProfile,
  parseProfileText,
} from "./services/profileManager.js";
import { updateTelemetrySubscription } from "./services/telemetryManager.js";
import { UsbSerialTransport } from "./services/usbSerialTransport.js";

const navItems = [
  "Dashboard",
  "Setup",
  "Sensors",
  "Receiver",
  "Motors",
  "Servos",
  "PID Tuning",
  "Rates",
  "Filters and FFT",
  "Compass",
  "GPS and Navigation",
  "AI Vision",
  "Telemetry",
  "Logs",
  "System",
  "Firmware",
  "Profiles",
];

const pidLabels = [
  "Roll Rate P", "Roll Rate I", "Roll Rate D",
  "Pitch Rate P", "Pitch Rate I", "Pitch Rate D",
  "Yaw Rate P", "Yaw Rate I", "Yaw Rate D",
  "Roll Angle P", "Pitch Angle P", "Yaw Angle P",
];

const activePage = ref("Dashboard");
const mode = ref("mock");
const ports = ref([]);
const selectedPort = ref("");
const client = ref(null);
const hello = ref(null);
const capabilities = ref(null);
const state = ref(null);
const sensors = ref(null);
const advanced = ref(null);
const config = ref(null);
const statusText = ref("Disconnected");
const errorText = ref("");
const busy = ref(false);
const telemetryStatus = ref("No subscription");
const profileText = ref("");
const selectedFirmwareFile = ref(null);
const firmwarePlan = ref(null);
const eventLog = ref([]);
const motorConsent = ref(false);
const motorRaw = ref(300);
const motorTimeoutMs = ref(350);
const pressedMotor = ref(null);
const activeMotor = ref(null);
const pollTimer = ref(null);
const motorKeepaliveTimer = ref(null);
const systemHoldAction = ref("");
let motorKeepaliveInFlight = false;
let lastPollErrorText = "";
let systemHoldTimer = null;
let telemetryRefreshInFlight = false;

const DEFAULT_REQUEST_TIMEOUT_MS = 1200;
const USB_REQUEST_TIMEOUT_MS = 3000;
const SYSTEM_HOLD_MS = 1200;

const connected = computed(() => client.value?.isConnected() ?? false);
const attitude = computed(() => {
  const a = state.value?.att ?? [0, 0, 0];
  return { roll: a[0] ?? 0, pitch: a[1] ?? 0, yaw: a[2] ?? 0 };
});
const batteryText = computed(() => {
  if (!state.value) return "--";
  return `${Number(state.value.bat ?? 0).toFixed(2)} V`;
});
const gpsText = computed(() => {
  const gps = state.value?.gps ?? [0, 0];
  return gps[0] ? `${gps[1]} sats` : "No fix";
});
const receiverRows = computed(() => {
  const rc = state.value?.rc ?? [];
  return [
    ["Link", rc[0] ? "Linked" : "No link"],
    ["Quality", rc[1] != null ? `${rc[1]}%` : "--"],
    ["RSSI delta", rc[2] != null ? `${rc[2]}` : "--"],
    ["Packet age", rc[3] != null ? `${rc[3]} ms` : "--"],
  ];
});
const healthRows = computed(() => {
  const h = advanced.value?.health ?? [];
  return [
    ["Free heap", `${h[0] ?? state.value?.heap ?? "--"} bytes`],
    ["Minimum heap", `${h[1] ?? "--"} bytes`],
    ["Flight overruns", h[2] ?? "--"],
    ["Radio overruns", h[3] ?? "--"],
    ["Sensor overruns", h[4] ?? "--"],
    ["Max flight loop", h[5] != null ? `${h[5]} us` : "--"],
    ["Uptime", h[6] != null ? `${Math.round(h[6] / 1000)} s` : "--"],
  ];
});
const servoRows = computed(() => {
  const s = advanced.value?.servo ?? sensors.value?.servo ?? [];
  return [
    ["Attached", s[0] ? "Yes" : "No"],
    ["Pan actual", s[1] != null ? `${s[1]} us` : "--"],
    ["Tilt actual", s[2] != null ? `${s[2]} us` : "--"],
    ["Pan target", s[3] != null ? `${s[3]} us` : "--"],
    ["Tilt target", s[4] != null ? `${s[4]} us` : "--"],
  ];
});
const notchRows = computed(() => {
  const n = advanced.value?.notch ?? [];
  return [
    ["Compiled", n[0] ? "Yes" : "No"],
    ["Enabled", n[1] ? "Yes" : "No"],
    ["Center", n[2] != null ? `${Number(n[2]).toFixed(1)} Hz` : "--"],
    ["Range", n[3] != null ? `${Number(n[3]).toFixed(1)}-${Number(n[4]).toFixed(1)} Hz` : "--"],
    ["Q", n[5] != null ? Number(n[5]).toFixed(2) : "--"],
    ["Analysis", n[6] ? "Running" : (n[7] ? "Done" : "Idle")],
    ["Recommendation", n[8] != null ? `${Number(n[8]).toFixed(1)} Hz @ ${Number(n[9] ?? 0).toFixed(2)}` : "--"],
  ];
});
const captureRows = computed(() => {
  const c = advanced.value?.capture ?? [];
  return [
    ["Active", c[0] ? "Yes" : "No"],
    ["Waiting for arm", c[1] ? "Yes" : "No"],
    ["Has data", c[2] ? "Yes" : "No"],
    ["Overflow", c[3] ? "Yes" : "No"],
    ["Samples", c[4] != null && c[5] != null ? `${c[4]} / ${c[5]}` : "--"],
    ["Effective rate", c[6] != null ? `${Number(c[6]).toFixed(1)} Hz` : "--"],
  ];
});

function addEvent(message) {
  const time = new Date().toLocaleTimeString();
  eventLog.value = [{ time, message }, ...eventLog.value].slice(0, 80);
}

function errorMessage(error) {
  return error?.message || String(error || "Unknown error");
}

function recordError(error, prefix = "Action failed") {
  const message = `${prefix}: ${errorMessage(error)}`;
  errorText.value = message;
  addEvent(message);
}

function requestTimeoutMs() {
  return mode.value === "usb" ? USB_REQUEST_TIMEOUT_MS : DEFAULT_REQUEST_TIMEOUT_MS;
}

function isTelemetryPendingText(text) {
  return /^Initial telemetry pending:|^Telemetry pending:/.test(text);
}

function clearTelemetryPending() {
  lastPollErrorText = "";
  if (isTelemetryPendingText(errorText.value)) {
    errorText.value = "";
  }
}

function recordTelemetryPending(error, prefix = "Telemetry pending") {
  const message = `${prefix}: ${errorMessage(error)}. Serial is still open; retrying.`;
  errorText.value = message;
  if (message !== lastPollErrorText) {
    addEvent(message);
    lastPollErrorText = message;
  }
}

function makeTransport() {
  if (mode.value === "usb") return new UsbSerialTransport();
  if (mode.value === "ble") return new BleTransport();
  return new MockTransport();
}

async function refreshPorts() {
  errorText.value = "";
  ports.value = [];
  const transport = makeTransport();
  try {
    ports.value = await transport.discover();
    if (!selectedPort.value && ports.value.length > 0) {
      selectedPort.value = ports.value[0].path;
    }
    addEvent(`Discovered ${ports.value.length} ${mode.value.toUpperCase()} device(s)`);
  } catch (error) {
    recordError(error, "Discovery failed");
  }
}

async function connect() {
  busy.value = true;
  errorText.value = "";
  let nextClient = null;
  try {
    const transport = makeTransport();
    nextClient = new FcuClient(transport, { timeoutMs: requestTimeoutMs() });
    hello.value = await nextClient.connect({ path: mode.value === "mock" ? "MOCK" : selectedPort.value });
    client.value = nextClient;
    statusText.value = `${hello.value?.transport ?? mode.value} connected`;
    addEvent(`Connected over ${hello.value?.transport ?? mode.value}`);
    try {
      capabilities.value = await nextClient.getCapabilities();
    } catch (error) {
      recordError(error, "Capabilities pending");
    }
    await updatePageSubscription();
    try {
      await refreshAll();
      clearTelemetryPending();
    } catch (error) {
      recordTelemetryPending(error, "Initial telemetry pending");
    }
    startPolling();
  } catch (error) {
    await nextClient?.disconnect().catch(() => {});
    if (client.value === nextClient) {
      client.value = null;
    }
    hello.value = null;
    statusText.value = "Connection failed";
    recordError(error, "Connection failed");
  } finally {
    busy.value = false;
  }
}

async function disconnect() {
  busy.value = true;
  errorText.value = "";
  try {
    await stopMotor();
    stopPolling();
    await client.value?.disconnect();
    client.value = null;
    hello.value = null;
    capabilities.value = null;
    lastPollErrorText = "";
    statusText.value = "Disconnected";
    telemetryStatus.value = "No subscription";
    addEvent("Disconnected");
  } catch (error) {
    recordError(error, "Disconnect failed");
  } finally {
    busy.value = false;
  }
}

function pollingPeriodMs() {
  if (activePage.value === "Dashboard" || activePage.value === "Receiver" || activePage.value === "Motors") return 250;
  if (activePage.value === "Sensors" || activePage.value === "Filters and FFT" || activePage.value === "Telemetry") return 500;
  return 1000;
}

function startPolling() {
  stopPolling();
  pollTimer.value = setInterval(() => {
    refreshTelemetry().catch((error) => {
      recordTelemetryPending(error);
    });
  }, pollingPeriodMs());
}

function stopPolling() {
  if (pollTimer.value != null) {
    clearInterval(pollTimer.value);
    pollTimer.value = null;
  }
}

async function refreshTelemetry() {
  if (!client.value?.isConnected() || busy.value || telemetryRefreshInFlight) return;
  telemetryRefreshInFlight = true;
  try {
    state.value = await client.value.getState();
    clearTelemetryPending();
    if (["Dashboard", "Servos", "Filters and FFT", "Telemetry", "System"].includes(activePage.value)) {
      advanced.value = await client.value.getAdvancedStatus().catch(() => advanced.value);
    }
  } finally {
    telemetryRefreshInFlight = false;
  }
}

async function refreshAll() {
  if (!client.value?.isConnected()) return;
  state.value = await client.value.getState();
  sensors.value = await client.value.getSensorStatus();
  advanced.value = await client.value.getAdvancedStatus().catch(() => advanced.value);
  config.value = await client.value.getConfig();
  clearTelemetryPending();
}

async function refreshAllFromUi(label = "Refreshed from FCU") {
  if (!client.value?.isConnected()) return;
  busy.value = true;
  errorText.value = "";
  try {
    await refreshAll();
    statusText.value = label;
    addEvent(label);
  } catch (error) {
    recordError(error, "Refresh failed");
  } finally {
    busy.value = false;
  }
}

async function updatePageSubscription() {
  if (!client.value?.isConnected()) return;
  try {
    const sub = await updateTelemetrySubscription(client.value, activePage.value);
    telemetryStatus.value = sub ? `Group ${sub.group} at ${sub.rateHz} Hz` : "No subscription";
  } catch (error) {
    telemetryStatus.value = error.message;
  }
}

async function applyPid() {
  if (!config.value?.pid) return;
  busy.value = true;
  errorText.value = "";
  try {
    await client.value.applyPid(config.value.pid.map((v) => Number(v)));
    await refreshAll();
    statusText.value = "PID applied to RAM";
    addEvent("PID applied to RAM");
  } catch (error) {
    recordError(error, "Apply PID failed");
  } finally {
    busy.value = false;
  }
}

async function savePid() {
  busy.value = true;
  errorText.value = "";
  try {
    await client.value.savePid();
    statusText.value = "PID saved to NVS";
    addEvent("PID saved to NVS");
  } catch (error) {
    recordError(error, "Save PID failed");
  } finally {
    busy.value = false;
  }
}

async function reloadPid() {
  busy.value = true;
  errorText.value = "";
  try {
    await client.value.reloadPid();
    config.value = await client.value.getConfig();
    statusText.value = "PID reloaded from NVS";
    addEvent("PID reloaded from NVS");
  } catch (error) {
    recordError(error, "Reload PID failed");
  } finally {
    busy.value = false;
  }
}

function normalizedMotorRaw() {
  const value = Number(motorRaw.value);
  if (!Number.isFinite(value)) return 300;
  return Math.max(0, Math.min(600, Math.round(value)));
}

function normalizedMotorTimeoutMs() {
  const value = Number(motorTimeoutMs.value);
  if (!Number.isFinite(value)) return 350;
  return Math.max(100, Math.min(750, Math.round(value)));
}

function motorKeepalivePeriodMs() {
  return Math.max(80, Math.min(200, normalizedMotorTimeoutMs() - 100));
}

function stopMotorKeepalive() {
  if (motorKeepaliveTimer.value != null) {
    clearInterval(motorKeepaliveTimer.value);
    motorKeepaliveTimer.value = null;
  }
  motorKeepaliveInFlight = false;
}

async function sendMotorSet(motor) {
  await client.value.setMotorTest(motor, normalizedMotorRaw(), normalizedMotorTimeoutMs());
}

function startMotorKeepalive(motor) {
  stopMotorKeepalive();
  motorKeepaliveTimer.value = setInterval(async () => {
    if (!client.value?.isConnected() || activeMotor.value !== motor || motorKeepaliveInFlight) {
      return;
    }
    motorKeepaliveInFlight = true;
    try {
      await sendMotorSet(motor);
    } catch (error) {
      recordError(error, "Motor keepalive failed");
      await stopMotor();
    } finally {
      motorKeepaliveInFlight = false;
    }
  }, motorKeepalivePeriodMs());
}

async function startMotor(motor, event) {
  event?.preventDefault();
  if (!client.value?.isConnected() || !motorConsent.value || busy.value) return;
  if (activeMotor.value != null && activeMotor.value !== motor) return;
  pressedMotor.value = motor;
  busy.value = true;
  errorText.value = "";
  try {
    await client.value.armMotorTest();
    if (pressedMotor.value !== motor) {
      await client.value.stopMotorTest();
      statusText.value = `Motor ${motor} start canceled`;
      return;
    }
    activeMotor.value = motor;
    await sendMotorSet(motor);
    if (pressedMotor.value !== motor) {
      await stopMotor();
      return;
    }
    startMotorKeepalive(motor);
    statusText.value = `Holding M${motor} at raw ${normalizedMotorRaw()}`;
  } catch (error) {
    recordError(error, "Motor start failed");
    pressedMotor.value = null;
    activeMotor.value = null;
    stopMotorKeepalive();
    await client.value?.stopMotorTest().catch(() => {});
  } finally {
    busy.value = false;
  }
}

async function stopMotor() {
  const motor = activeMotor.value;
  pressedMotor.value = null;
  if (motor == null) {
    stopMotorKeepalive();
    return;
  }
  activeMotor.value = null;
  stopMotorKeepalive();
  if (!client.value?.isConnected()) return;
  busy.value = true;
  try {
    await client.value.stopMotorTest();
    statusText.value = `Motor ${motor} stopped`;
  } catch (error) {
    recordError(error, "Motor stop failed");
  } finally {
    busy.value = false;
  }
}

function sensorRows() {
  const s = sensors.value ?? {};
  return [
    ["IMU", "SPI", s.imu?.[1], s.imu?.[2], ""],
    ["Barometer", "I2C", s.baro?.[1], s.baro?.[2], `${s.baro?.[3] ?? "--"} ms`],
    ["Bottom ToF", "I2C", s.tof?.[1], s.tof?.[2], `${s.tof?.[4] ?? "--"} mm`],
    ["GPS", "UART1", s.gps?.[1], s.gps?.[2], `${s.gps?.[4] ?? 0} sats`],
    ["Magnetometer", "ICM20948", s.mag?.[1], s.mag?.[2], `${s.mag?.[3] ?? "--"} uT`],
    ["Battery", "ADC", s.batt?.[0], s.batt?.[0], `${s.batt?.[1] ?? "--"} V`],
    ["Companion", "UART2", s.pi?.[0], s.pi?.[1], s.pi?.[1] ? "alive" : "not found"],
    ["Pan/Tilt Servo", "LEDC", s.servo?.[0], s.servo?.[0], `${s.servo?.[1] ?? "--"} / ${s.servo?.[2] ?? "--"} us`],
  ];
}

async function rescanSensors() {
  if (!client.value?.isConnected()) return;
  busy.value = true;
  errorText.value = "";
  try {
    await client.value.rescanSensors();
    statusText.value = "Sensor rescan queued";
    addEvent("Sensor rescan queued");
    setTimeout(() => refreshAll().catch(() => {}), 600);
  } catch (error) {
    recordError(error, "Sensor rescan rejected");
  } finally {
    busy.value = false;
  }
}

async function runCalibration(id, label) {
  if (!client.value?.isConnected()) return;
  busy.value = true;
  errorText.value = "";
  try {
    await client.value.calibrate(id);
    statusText.value = `${label} accepted`;
    addEvent(`${label} accepted`);
  } catch (error) {
    recordError(error, `${label} rejected`);
  } finally {
    busy.value = false;
  }
}

function channelWidth(value) {
  const pct = Number(value);
  if (!Number.isFinite(pct)) return 0;
  return Math.max(0, Math.min(100, pct));
}

function pidAxisRows() {
  const pid = config.value?.pid ?? [];
  return [
    ["Roll rate", pid[0], pid[1], pid[2]],
    ["Pitch rate", pid[3], pid[4], pid[5]],
    ["Yaw rate", pid[6], pid[7], pid[8]],
    ["Roll angle", pid[9], "--", "--"],
    ["Pitch angle", pid[10], "--", "--"],
    ["Yaw angle", pid[11], "--", "--"],
  ];
}

function gpsRows() {
  const gps = sensors.value?.gps ?? [];
  return [
    ["Compiled", gps[0] ? "Yes" : "No"],
    ["Detected", gps[1] ? "Yes" : "No"],
    ["Fix", gps[2] ? "3D" : "No fix"],
    ["Age", gps[3] != null ? `${gps[3]} ms` : "--"],
    ["Satellites", gps[4] ?? state.value?.gps?.[1] ?? "--"],
  ];
}

function aiRows() {
  const pi = sensors.value?.pi ?? [];
  return [
    ["Companion link", pi[1] ? "Alive" : "Not found"],
    ["Autonomy serial", pi[0] ? "Compiled" : "Disabled"],
    ["Last packet age", pi[2] === 4294967295 ? "Never" : `${pi[2] ?? "--"} ms`],
    ["Authority", "Observer only"],
  ];
}

function telemetryRows() {
  const stats = client.value?.stats ?? {};
  return [
    ["Subscription", telemetryStatus.value],
    ["RX frames", stats.rxFrames ?? 0],
    ["TX frames", stats.txFrames ?? 0],
    ["Timeouts", stats.timeouts ?? 0],
    ["CRC errors", stats.crcErrors ?? 0],
    ["Transport BLE", capabilities.value?.transportBle ? "Yes" : "No"],
  ];
}

async function exportProfile() {
  busy.value = true;
  errorText.value = "";
  try {
    const profile = client.value?.isConnected()
      ? await client.value.exportProfile()
      : createProfileFromConfig({ config: config.value, metadata: { source: "offline" } });
    profileText.value = formatProfile(profile);
    statusText.value = "Profile exported";
    addEvent("Profile exported");
  } catch (error) {
    recordError(error, "Profile export failed");
  } finally {
    busy.value = false;
  }
}

async function applyProfileText() {
  busy.value = true;
  errorText.value = "";
  try {
    const profile = parseProfileText(profileText.value);
    config.value = {
      ...(config.value ?? {}),
      pid: profile.pid.slice(),
      mixBias: Number(profile.mixBias ?? 1.0),
    };
    if (client.value?.isConnected()) {
      await client.value.applyPid(profile.pid);
      await client.value.applyMixBias(Number(profile.mixBias ?? 1.0));
    }
    statusText.value = "Profile applied to RAM";
    addEvent("Profile applied to RAM");
  } catch (error) {
    recordError(error, "Profile apply failed");
  } finally {
    busy.value = false;
  }
}

async function selectFirmware(event) {
  const file = event.target.files?.[0] ?? null;
  selectedFirmwareFile.value = file;
  firmwarePlan.value = null;
  if (!file) return;
  try {
    const prepared = await prepareFirmwareFile(file);
    firmwarePlan.value = prepared.metadata;
  } catch (error) {
    recordError(error, "Firmware file rejected");
  }
}

async function stageFirmwareUpdate() {
  if (!client.value?.isConnected()) return;
  busy.value = true;
  errorText.value = "";
  try {
    const prepared = await prepareFirmwareFile(selectedFirmwareFile.value);
    firmwarePlan.value = prepared.metadata;
    await client.value.beginFirmwareUpdate(prepared.metadata);
    statusText.value = "Firmware update started";
  } catch (error) {
    recordError(error, "Firmware update unavailable");
  } finally {
    busy.value = false;
  }
}

async function systemAction(action) {
  if (!client.value?.isConnected()) return;
  busy.value = true;
  errorText.value = "";
  try {
    if (action === "reboot") {
      await client.value.reboot();
    } else {
      try {
        await client.value.enterBootloader();
      } catch (error) {
        const touchBootloader = client.value?.transport?.touchBootloader;
        if (mode.value !== "usb" || typeof touchBootloader !== "function") {
          throw error;
        }
        await touchBootloader.call(client.value.transport);
      }
    }
    statusText.value = `${action} accepted`;
    addEvent(`${action} accepted`);
    if (action === "reboot" || action === "bootloader") {
      stopPolling();
    }
  } catch (error) {
    recordError(error, `${action} rejected`);
  } finally {
    busy.value = false;
  }
}

function clearSystemHold() {
  if (systemHoldTimer != null) {
    clearTimeout(systemHoldTimer);
    systemHoldTimer = null;
  }
  systemHoldAction.value = "";
}

function startSystemHold(action, event) {
  event?.preventDefault();
  if (!client.value?.isConnected() || busy.value || systemHoldTimer != null) return;
  systemHoldAction.value = action;
  systemHoldTimer = setTimeout(() => {
    systemHoldTimer = null;
    systemHoldAction.value = "";
    systemAction(action);
  }, SYSTEM_HOLD_MS);
}

function cancelSystemHold(action) {
  if (!systemHoldAction.value || systemHoldAction.value !== action) return;
  clearSystemHold();
}

watch(activePage, () => {
  if (!client.value?.isConnected()) return;
  startPolling();
  updatePageSubscription().catch(() => {});
  refreshTelemetry().catch(() => {});
});

onMounted(() => {
  globalThis.addEventListener?.("error", (event) => {
    recordError(event.error || event.message, "App error");
    event.preventDefault?.();
  });
  globalThis.addEventListener?.("unhandledrejection", (event) => {
    recordError(event.reason, "Unhandled action error");
    event.preventDefault?.();
  });
});

onBeforeUnmount(() => {
  stopPolling();
  stopMotorKeepalive();
  clearSystemHold();
  pressedMotor.value = null;
  if (client.value?.isConnected()) {
    client.value.stopMotorTest().catch(() => {});
  }
});
</script>

<template>
  <div class="app-shell">
    <aside class="sidebar">
      <div class="brand">
        <img src="./assets/aeroforge-icon.svg" alt="" />
        <div>
          <strong>{{ appIdentity.shortName }}</strong>
          <span>FCU Configurator</span>
        </div>
      </div>
      <button
        v-for="item in navItems"
        :key="item"
        class="nav-item"
        :class="{ active: activePage === item }"
        @click="activePage = item"
      >
        {{ item }}
      </button>
    </aside>

    <main class="workspace">
      <header class="topbar">
        <div class="device-strip">
          <span class="pill" :class="connected ? 'ok' : 'warn'">{{ statusText }}</span>
          <span>Target: {{ hello?.target ?? appIdentity.hardwareTarget }}</span>
          <span>Protocol: {{ hello?.proto ?? capabilities?.proto ?? "--" }}</span>
          <span>Battery: {{ batteryText }}</span>
          <span :class="state?.fs ? 'bad-text' : ''">Failsafe: {{ state?.fs ? "Active" : "Clear" }}</span>
          <span>GPS: {{ gpsText }}</span>
        </div>
        <div class="connection-controls">
          <select v-model="mode" :disabled="connected">
            <option value="mock">Mock FCU</option>
            <option value="usb">USB Serial</option>
            <option value="ble">Bluetooth LE</option>
          </select>
          <select v-if="mode === 'usb'" v-model="selectedPort" :disabled="connected">
            <option value="">Select USB device</option>
            <option v-for="port in ports" :key="port.path" :value="port.path">
              {{ port.path }} - {{ port.friendlyName || port.manufacturer || "Serial device" }}
            </option>
          </select>
          <button v-if="mode === 'usb'" @click="refreshPorts" :disabled="busy || connected">Refresh USB</button>
          <button v-if="!connected" class="primary" @click="connect" :disabled="busy || (mode === 'usb' && !selectedPort)">Connect</button>
          <button v-else @click="disconnect" :disabled="busy">Disconnect</button>
        </div>
      </header>

      <div v-if="errorText" class="error-panel">{{ errorText }}</div>

      <section v-if="activePage === 'Dashboard'" class="page dashboard-grid">
        <div class="hero-panel">
          <AttitudeScene :roll="attitude.roll" :pitch="attitude.pitch" :yaw="attitude.yaw" />
        </div>
        <div class="metric-grid">
          <div class="metric"><span>Roll</span><strong>{{ attitude.roll.toFixed(2) }} deg</strong></div>
          <div class="metric"><span>Pitch</span><strong>{{ attitude.pitch.toFixed(2) }} deg</strong></div>
          <div class="metric"><span>Yaw</span><strong>{{ attitude.yaw.toFixed(2) }} deg</strong></div>
          <div class="metric"><span>Loop</span><strong>{{ state?.loop ?? "--" }} Hz</strong></div>
          <div class="metric"><span>Receiver</span><strong>{{ state?.rc?.[0] ? "Linked" : "No link" }}</strong></div>
          <div class="metric"><span>Heap</span><strong>{{ state?.heap ?? "--" }}</strong></div>
        </div>
        <div class="wide-panel">
          <h2>Motor Outputs</h2>
          <div class="motor-bars">
            <div v-for="(value, index) in (state?.mot ?? [0,0,0,0])" :key="index" class="motor-bar">
              <span>M{{ index + 1 }}</span>
              <div><i :style="{ width: `${Math.min(100, value / 20.47)}%` }"></i></div>
              <strong>{{ value }}</strong>
            </div>
          </div>
        </div>
      </section>

      <section v-else-if="activePage === 'Setup'" class="page two-column">
        <div class="panel">
          <h2>Connection</h2>
          <p class="muted">USB and Bluetooth LE use the same binary protocol. BLE appears only when the Windows runtime exposes Web Bluetooth and the FCU BLE endpoint is enabled.</p>
          <button @click="refreshPorts" :disabled="busy">Refresh USB Devices</button>
          <div class="port-list">
            <div v-for="port in ports" :key="port.path" class="port-row">
              <strong>{{ port.path }}</strong>
              <span>{{ port.friendlyName || port.manufacturer || "Serial device" }}</span>
              <small>VID {{ port.vendorId || "--" }} PID {{ port.productId || "--" }}</small>
            </div>
          </div>
        </div>
        <div class="panel">
          <h2>Device</h2>
          <pre>{{ JSON.stringify({ hello, capabilities }, null, 2) }}</pre>
        </div>
      </section>

      <section v-else-if="activePage === 'Sensors'" class="page">
        <div class="panel">
          <div class="panel-head">
            <h2>Sensor Status</h2>
            <div class="actions inline">
              <button @click="refreshAllFromUi('Sensor status refreshed')" :disabled="!connected || busy">Refresh</button>
              <button @click="rescanSensors" :disabled="!connected || busy || !capabilities?.sensorRescan">Rescan I2C</button>
            </div>
          </div>
          <table>
            <thead><tr><th>Device</th><th>Bus</th><th>Detected</th><th>Valid</th><th>Value</th></tr></thead>
            <tbody>
              <tr v-for="row in sensorRows()" :key="row[0]">
                <td>{{ row[0] }}</td>
                <td>{{ row[1] }}</td>
                <td><span class="pill" :class="row[2] ? 'ok' : 'warn'">{{ row[2] ? "Connected" : "Not found" }}</span></td>
                <td>{{ row[3] ? "Valid" : "Invalid" }}</td>
                <td>{{ row[4] }}</td>
              </tr>
            </tbody>
          </table>
          <p class="muted">Software rescanning does not guarantee that a sensor is electrically safe to connect while the FCU is powered.</p>
        </div>
      </section>

      <section v-else-if="activePage === 'Receiver'" class="page two-column">
        <div class="panel">
          <h2>Receiver Link</h2>
          <div class="kv-list">
            <div v-for="row in receiverRows" :key="row[0]"><span>{{ row[0] }}</span><strong>{{ row[1] }}</strong></div>
          </div>
        </div>
        <div class="panel">
          <h2>Live Link Quality</h2>
          <div class="channel-list">
            <div class="channel-row">
              <span>Quality</span>
              <div class="bar"><i :style="{ width: `${channelWidth(state?.rc?.[1])}%` }"></i></div>
              <strong>{{ state?.rc?.[1] ?? "--" }}%</strong>
            </div>
            <div class="channel-row">
              <span>Throttle</span>
              <div class="bar warn"><i :style="{ width: `${channelWidth(state?.thr)}%` }"></i></div>
              <strong>{{ state?.thr ?? "--" }}%</strong>
            </div>
          </div>
          <p class="muted">Manual RC remains the authority path; this page only observes link health.</p>
        </div>
      </section>

      <section v-else-if="activePage === 'PID Tuning'" class="page">
        <div class="panel">
          <div class="panel-head">
            <h2>PID Tuning</h2>
            <button @click="refreshAllFromUi('PID reloaded from FCU')" :disabled="!connected || busy">Reload From FCU</button>
          </div>
          <div v-if="config?.pid" class="pid-grid">
            <label v-for="(label, index) in pidLabels" :key="label">
              <span>{{ label }}</span>
              <input v-model.number="config.pid[index]" type="number" step="1" />
            </label>
          </div>
          <div class="actions">
            <button class="primary" @click="applyPid" :disabled="!connected || busy || !config?.pid">Apply RAM</button>
            <button @click="savePid" :disabled="!connected || busy">Save to NVS</button>
            <button @click="reloadPid" :disabled="!connected || busy">Revert from NVS</button>
          </div>
        </div>
      </section>

      <section v-else-if="activePage === 'Motors'" class="page">
        <div class="panel danger">
          <h2>Motor Test</h2>
          <p>Deadman motor test is accepted only while disarmed, throttle zero, idle, failsafe clear, and no other bench action is active. Hold a motor button to keep sending short-timeout commands; release to stop.</p>
          <label class="check"><input v-model="motorConsent" type="checkbox" /> Props removed and frame restrained</label>
          <div class="motor-settings">
            <label>
              <span>Raw command</span>
              <input v-model.number="motorRaw" type="number" min="0" max="600" step="10" />
            </label>
            <label>
              <span>Timeout ms</span>
              <input v-model.number="motorTimeoutMs" type="number" min="100" max="750" step="50" />
            </label>
          </div>
          <div class="motor-buttons">
            <button
              v-for="motor in [1,2,3,4]"
              :key="motor"
              class="hold-button"
              :class="{ active: activeMotor === motor }"
              :disabled="!connected || !motorConsent || (activeMotor != null && activeMotor !== motor)"
              @pointerdown="startMotor(motor, $event)"
              @pointerup="stopMotor"
              @pointercancel="stopMotor"
              @pointerleave="stopMotor"
              @contextmenu.prevent
            >
              {{ activeMotor === motor ? `Holding M${motor}` : `Hold M${motor}` }}
            </button>
          </div>
          <p v-if="activeMotor" class="muted">Keepalive every {{ motorKeepalivePeriodMs() }} ms; FCU output timeout {{ normalizedMotorTimeoutMs() }} ms.</p>
        </div>
      </section>

      <section v-else-if="activePage === 'Servos'" class="page two-column">
        <div class="panel">
          <h2>Pan/Tilt Servo</h2>
          <div class="kv-list">
            <div v-for="row in servoRows" :key="row[0]"><span>{{ row[0] }}</span><strong>{{ row[1] }}</strong></div>
          </div>
        </div>
        <div class="panel">
          <h2>Safety</h2>
          <p class="muted">This page is read-only in the Windows configurator stage. Servo writes still go through existing firmware callbacks after the safety policy is expanded.</p>
        </div>
      </section>

      <section v-else-if="activePage === 'Rates'" class="page">
        <div class="panel">
          <div class="panel-head">
            <h2>Rate and Angle Gains</h2>
            <button @click="refreshAllFromUi('Rates reloaded from FCU')" :disabled="!connected || busy">Reload</button>
          </div>
          <table>
            <thead><tr><th>Axis</th><th>P</th><th>I</th><th>D</th></tr></thead>
            <tbody>
              <tr v-for="row in pidAxisRows()" :key="row[0]">
                <td>{{ row[0] }}</td><td>{{ row[1] }}</td><td>{{ row[2] }}</td><td>{{ row[3] }}</td>
              </tr>
            </tbody>
          </table>
        </div>
      </section>

      <section v-else-if="activePage === 'Filters and FFT'" class="page two-column">
        <div class="panel">
          <h2>Dynamic Notch</h2>
          <div class="kv-list">
            <div v-for="row in notchRows" :key="row[0]"><span>{{ row[0] }}</span><strong>{{ row[1] }}</strong></div>
          </div>
        </div>
        <div class="panel">
          <h2>Capture Buffer</h2>
          <div class="kv-list">
            <div v-for="row in captureRows" :key="row[0]"><span>{{ row[0] }}</span><strong>{{ row[1] }}</strong></div>
          </div>
        </div>
      </section>

      <section v-else-if="activePage === 'Compass'" class="page two-column">
        <div class="panel">
          <h2>Magnetometer</h2>
          <div class="kv-list">
            <div><span>Compiled</span><strong>{{ sensors?.mag?.[0] ? "Yes" : "No" }}</strong></div>
            <div><span>Detected</span><strong>{{ sensors?.mag?.[1] ? "Yes" : "No" }}</strong></div>
            <div><span>Healthy</span><strong>{{ sensors?.mag?.[2] ? "Yes" : "No" }}</strong></div>
            <div><span>Magnitude</span><strong>{{ sensors?.mag?.[3] ?? "--" }} uT</strong></div>
            <div><span>Yaw</span><strong>{{ attitude.yaw.toFixed(1) }} deg</strong></div>
          </div>
        </div>
        <div class="panel">
          <h2>Calibration</h2>
          <div class="actions">
            <button @click="runCalibration(3, 'Mag calibration start')" :disabled="!connected || busy">Start Mag Cal</button>
            <button @click="runCalibration(4, 'Mag calibration finish')" :disabled="!connected || busy">Finish Mag Cal</button>
          </div>
        </div>
      </section>

      <section v-else-if="activePage === 'GPS and Navigation'" class="page two-column">
        <div class="panel">
          <h2>GPS</h2>
          <div class="kv-list">
            <div v-for="row in gpsRows()" :key="row[0]"><span>{{ row[0] }}</span><strong>{{ row[1] }}</strong></div>
          </div>
        </div>
        <div class="panel">
          <h2>Navigation Authority</h2>
          <p class="muted">Navigation settings remain observational here. The configurator does not promote autonomy into motor authority.</p>
          <div class="kv-list">
            <div><span>Failsafe</span><strong>{{ state?.fs ? "Active" : "Clear" }}</strong></div>
            <div><span>Armed</span><strong>{{ state?.arm ? "Yes" : "No" }}</strong></div>
            <div><span>ToF</span><strong>{{ state?.tof ?? "--" }} mm</strong></div>
          </div>
        </div>
      </section>

      <section v-else-if="activePage === 'AI Vision'" class="page two-column">
        <div class="panel">
          <h2>Companion Link</h2>
          <div class="kv-list">
            <div v-for="row in aiRows()" :key="row[0]"><span>{{ row[0] }}</span><strong>{{ row[1] }}</strong></div>
          </div>
        </div>
        <div class="panel">
          <h2>Decision Channel</h2>
          <p class="muted">AI/vision data is shown as telemetry only. Manual RC override and FCU failsafes stay ahead of companion decisions.</p>
        </div>
      </section>

      <section v-else-if="activePage === 'Telemetry'" class="page two-column">
        <div class="panel">
          <h2>Protocol Stats</h2>
          <div class="kv-list">
            <div v-for="row in telemetryRows()" :key="row[0]"><span>{{ row[0] }}</span><strong>{{ row[1] }}</strong></div>
          </div>
        </div>
        <div class="panel">
          <h2>Capabilities</h2>
          <pre>{{ JSON.stringify(capabilities, null, 2) }}</pre>
        </div>
      </section>

      <section v-else-if="activePage === 'Logs'" class="page">
        <div class="panel">
          <h2>Session Events</h2>
          <div class="log-list">
            <div v-for="entry in eventLog" :key="`${entry.time}-${entry.message}`">
              <span>{{ entry.time }}</span><code>{{ entry.message }}</code>
            </div>
          </div>
          <p class="muted">Framed runtime log streaming stays disabled in the configurator firmware target until log isolation is hardware-verified.</p>
        </div>
      </section>

      <section v-else-if="activePage === 'System'" class="page two-column">
        <div class="panel">
          <h2>Task Health</h2>
          <div class="kv-list">
            <div v-for="row in healthRows" :key="row[0]"><span>{{ row[0] }}</span><strong>{{ row[1] }}</strong></div>
          </div>
        </div>
        <div class="panel danger">
          <h2>System Actions</h2>
          <p class="muted">Hold to request a reboot or USB download boot. The FCU rejects these unless it is bench-idle.</p>
          <div class="actions">
            <button
              class="hold-button system-hold"
              :class="{ active: systemHoldAction === 'reboot' }"
              :disabled="!connected || busy"
              @pointerdown="startSystemHold('reboot', $event)"
              @pointerup="cancelSystemHold('reboot')"
              @pointercancel="cancelSystemHold('reboot')"
              @pointerleave="cancelSystemHold('reboot')"
              @contextmenu.prevent
            >
              {{ systemHoldAction === "reboot" ? "Holding Reboot" : "Hold Reboot" }}
            </button>
            <button
              class="hold-button system-hold"
              :class="{ active: systemHoldAction === 'bootloader' }"
              :disabled="!connected || busy"
              @pointerdown="startSystemHold('bootloader', $event)"
              @pointerup="cancelSystemHold('bootloader')"
              @pointercancel="cancelSystemHold('bootloader')"
              @pointerleave="cancelSystemHold('bootloader')"
              @contextmenu.prevent
            >
              {{ systemHoldAction === "bootloader" ? "Holding Bootloader" : "Hold Bootloader" }}
            </button>
          </div>
          <p class="muted">Hold for {{ SYSTEM_HOLD_MS / 1000 }} seconds. The FCU accepts only while bench-idle: disarmed, throttle zero, no motor output.</p>
        </div>
      </section>

      <section v-else-if="activePage === 'Firmware'" class="page two-column">
        <div class="panel">
          <h2>Firmware Package</h2>
          <input type="file" accept=".bin" @change="selectFirmware" />
          <div v-if="firmwarePlan" class="kv-list">
            <div><span>Name</span><strong>{{ firmwarePlan.name }}</strong></div>
            <div><span>Size</span><strong>{{ firmwarePlan.size }} bytes</strong></div>
            <div><span>Checksum</span><strong>{{ firmwarePlan.checksum }}</strong></div>
          </div>
          <button class="primary" @click="stageFirmwareUpdate" :disabled="!connected || busy || !selectedFirmwareFile">Start Update</button>
        </div>
        <div class="panel">
          <h2>Update Status</h2>
          <p class="muted">The command IDs and file validation are wired; firmware responds not supported until a signed OTA/USB writer is enabled.</p>
        </div>
      </section>

      <section v-else-if="activePage === 'Profiles'" class="page two-column">
        <div class="panel">
          <div class="panel-head">
            <h2>Profile JSON</h2>
            <button @click="exportProfile" :disabled="busy || (!connected && !config?.pid)">Export</button>
          </div>
          <textarea v-model="profileText" spellcheck="false"></textarea>
          <div class="actions">
            <button class="primary" @click="applyProfileText" :disabled="busy || !profileText">Apply to RAM</button>
            <button @click="savePid" :disabled="!connected || busy">Save PID to NVS</button>
          </div>
        </div>
        <div class="panel">
          <h2>Active Profile</h2>
          <pre>{{ JSON.stringify({ pid: config?.pid, mixBias: config?.mixBias }, null, 2) }}</pre>
        </div>
      </section>

      <section v-else class="page">
        <div class="panel empty-state">
          <h2>{{ activePage }}</h2>
          <p>Unknown page selection. Use the sidebar to return to a supported configurator surface.</p>
        </div>
      </section>
    </main>
  </div>
</template>
