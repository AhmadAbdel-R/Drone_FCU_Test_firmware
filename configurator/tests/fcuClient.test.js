import assert from "node:assert/strict";
import test from "node:test";

import { FcuClient } from "../src/services/fcuClient.js";
import { MockTransport } from "../src/services/mockTransport.js";
import { StatusCode } from "../src/services/protocol.js";

test("FcuClient completes mock handshake and reads state", async () => {
  const client = new FcuClient(new MockTransport());
  const hello = await client.connect({ path: "MOCK" });
  assert.equal(hello.target, "esp32-s3-mini");

  const capabilities = await client.getCapabilities();
  assert.equal(capabilities.pid, true);
  assert.equal(capabilities.motorTest, "deadman");

  const state = await client.getState();
  assert.equal(state.loop, 500);
  assert.equal(Array.isArray(state.att), true);

  await client.disconnect();
});

test("FcuClient reads, applies, saves, and reloads PID config on mock FCU", async () => {
  const client = new FcuClient(new MockTransport());
  await client.connect({ path: "MOCK" });

  const config = await client.getConfig();
  assert.equal(config.pid.length, 12);
  const next = config.pid.slice();
  next[3] += 10;

  await client.applyPid(next);
  await client.savePid();
  await client.reloadPid();

  const updated = await client.getConfig();
  assert.deepEqual(updated.pid, next);
  await client.disconnect();
});

test("FcuClient sends mock deadman motor test session", async () => {
  const transport = new MockTransport();
  const client = new FcuClient(transport);
  await client.connect({ path: "MOCK" });
  await client.armMotorTest();
  await client.setMotorTest(2, 320, 350);
  assert.equal(transport.motorSetCount, 1);
  assert.deepEqual(transport.lastMotorCommand, {
    motorMask: 0x02,
    raw: 320,
    timeoutMs: 350,
  });
  await client.stopMotorTest();
  assert.equal(transport.motorStopCount, 1);
  assert.equal(transport.lastMotorCommand, null);
  await client.disconnect();
});

test("FcuClient reads advanced status and exports profile", async () => {
  const client = new FcuClient(new MockTransport());
  await client.connect({ path: "MOCK" });

  const advanced = await client.getAdvancedStatus();
  assert.equal(Array.isArray(advanced.health), true);
  assert.equal(Array.isArray(advanced.notch), true);

  const profile = await client.exportProfile();
  assert.equal(profile.schema, "aeroforge.profile.v1");
  assert.equal(profile.pid.length, 12);

  await client.disconnect();
});

test("FcuClient handles rescan, telemetry subscribe, and disabled firmware update", async () => {
  const client = new FcuClient(new MockTransport());
  await client.connect({ path: "MOCK" });

  await client.rescanSensors();
  await client.subscribeTelemetry(1, 10);

  await assert.rejects(
    () => client.beginFirmwareUpdate({ name: "fcu.bin", size: 1024 }),
    (error) => error.status === StatusCode.NOT_SUPPORTED,
  );

  await client.disconnect();
});
