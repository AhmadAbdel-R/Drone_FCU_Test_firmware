import assert from "node:assert/strict";
import test from "node:test";

import { UsbSerialTransport } from "../src/services/usbSerialTransport.js";

function replaceGlobal(name, value) {
  const previous = Object.getOwnPropertyDescriptor(globalThis, name);
  Object.defineProperty(globalThis, name, {
    configurable: true,
    value,
  });
  return () => {
    if (previous) {
      Object.defineProperty(globalThis, name, previous);
    } else {
      delete globalThis[name];
    }
  };
}

test("USB discovery uses Web Serial picker without native serial", async () => {
  const restoreNavigator = replaceGlobal("navigator", {
    serial: {
      getPorts: async () => [{
        getInfo: () => ({ usbVendorId: 0x303a, usbProductId: 0x1001 }),
      }],
    },
  });

  try {
    const ports = await new UsbSerialTransport().discover();
    assert.equal(ports.length, 2);
    assert.equal(ports[0].path, "WEB_SERIAL:0");
    assert.equal(ports[0].likelyEsp32, true);
    assert.equal(ports[1].path, "WEB_SERIAL_PICKER");
  } finally {
    restoreNavigator();
  }
});

test("USB discovery fails safely when Web Serial and bridge are unavailable", async () => {
  const restoreNavigator = replaceGlobal("navigator", {});
  const restoreStorage = replaceGlobal("localStorage", {
    getItem: () => null,
  });

  try {
    await assert.rejects(
      () => new UsbSerialTransport().discover(),
      /COM listing needs/,
    );
  } finally {
    restoreStorage();
    restoreNavigator();
  }
});
