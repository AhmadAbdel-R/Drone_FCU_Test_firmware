import { Transport } from "./transport.js";
import { BleGatt, DEFAULT_BLE_CHUNK_LENGTH, fragmentBleFrame } from "./protocol.js";

export class BleTransport extends Transport {
  constructor() {
    super("Bluetooth LE");
    this.device = null;
    this.server = null;
    this.rx = null;
    this.tx = null;
    this.info = null;
    this.onDisconnected = null;
  }

  async discover() {
    if (!globalThis.navigator?.bluetooth) {
      return [];
    }
    return [{
      path: "BLE_PICKER",
      friendlyName: "Open Windows Bluetooth picker",
      manufacturer: "Bluetooth LE",
      serialNumber: "",
      transport: "ble",
    }];
  }

  async connect() {
    const bluetooth = globalThis.navigator?.bluetooth;
    if (!bluetooth?.requestDevice) {
      throw new Error("Bluetooth LE requires the NW.js/Chromium runtime with Web Bluetooth enabled on Windows.");
    }

    this.device = await bluetooth.requestDevice({
      filters: [{ services: [BleGatt.SERVICE_UUID] }],
      optionalServices: [BleGatt.SERVICE_UUID],
    });
    this.onDisconnected = () => this.setConnected(false, {
      path: this.device?.id ?? "BLE",
      friendlyName: this.device?.name ?? "AeroForge FCU",
    });
    this.device.addEventListener("gattserverdisconnected", this.onDisconnected);
    this.server = await this.device.gatt.connect();
    const service = await this.server.getPrimaryService(BleGatt.SERVICE_UUID);
    this.rx = await service.getCharacteristic(BleGatt.RX_UUID);
    this.tx = await service.getCharacteristic(BleGatt.TX_UUID);
    this.info = await service.getCharacteristic(BleGatt.INFO_UUID).catch(() => null);
    this.tx.addEventListener("characteristicvaluechanged", (event) => {
      const view = event.target.value;
      this.emitData(Buffer.from(view.buffer, view.byteOffset, view.byteLength));
    });
    await this.tx.startNotifications();
    this.setConnected(true, {
      path: this.device.id ?? "BLE",
      friendlyName: this.device.name ?? "AeroForge FCU",
      transport: "ble",
    });
  }

  async disconnect() {
    if (this.tx) {
      await this.tx.stopNotifications().catch(() => {});
    }
    if (this.device && this.onDisconnected) {
      this.device.removeEventListener("gattserverdisconnected", this.onDisconnected);
    }
    if (this.device?.gatt?.connected) {
      this.device.gatt.disconnect();
    }
    this.device = null;
    this.server = null;
    this.rx = null;
    this.tx = null;
    this.info = null;
    this.onDisconnected = null;
    this.setConnected(false);
  }

  async write(data) {
    if (!this.rx || !this.connected) {
      throw new Error("Bluetooth LE transport is not connected");
    }
    for (const chunk of fragmentBleFrame(data, DEFAULT_BLE_CHUNK_LENGTH)) {
      const value = Uint8Array.from(chunk);
      if (typeof this.rx.writeValueWithoutResponse === "function") {
        await this.rx.writeValueWithoutResponse(value);
      } else {
        await this.rx.writeValue(value);
      }
    }
  }
}
