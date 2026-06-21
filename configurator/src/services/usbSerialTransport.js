import { Transport } from "./transport.js";

const WEB_SERIAL_PICKER_PATH = "WEB_SERIAL_PICKER";
const ESP_USB_VENDOR_IDS = new Set([0x303a, 0x10c4, 0x1a86]);

function nodeRequire() {
  return globalThis.window?.require ?? globalThis.require ?? null;
}

function serialApi() {
  return globalThis.navigator?.serial ?? null;
}

function delay(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

function likelyEsp32(vendorId, productId) {
  const numericVendorId = typeof vendorId === "number"
    ? vendorId
    : Number.parseInt(String(vendorId), 16);
  return ESP_USB_VENDOR_IDS.has(numericVendorId) ||
    /303a|10c4|1a86/i.test(`${vendorId ?? ""}:${productId ?? ""}`);
}

function webSerialPortInfo(port, index) {
  const info = port.getInfo?.() ?? {};
  const usbVendorId = info.usbVendorId ?? "";
  const usbProductId = info.usbProductId ?? "";
  return {
    path: `WEB_SERIAL:${index}`,
    friendlyName: likelyEsp32(usbVendorId, usbProductId) ? "Authorized ESP32 serial device" : "Authorized USB serial device",
    manufacturer: "Web Serial",
    serialNumber: "",
    vendorId: usbVendorId === "" ? "" : usbVendorId.toString(16).padStart(4, "0"),
    productId: usbProductId === "" ? "" : usbProductId.toString(16).padStart(4, "0"),
    transport: "web-serial",
    likelyEsp32: likelyEsp32(usbVendorId, usbProductId),
  };
}

function bridgeRuntime() {
  const req = nodeRequire();
  if (typeof req !== "function") return null;
  const fs = req("fs");
  const path = req("path");
  const childProcess = req("child_process");
  const cwd = globalThis.process?.cwd?.() ?? "";
  const exeDir = globalThis.process?.execPath ? path.dirname(globalThis.process.execPath) : "";
  const candidates = [
    path.join(cwd, "serial-bridge", "serial-bridge.cjs"),
    path.join(exeDir, "serial-bridge", "serial-bridge.cjs"),
    path.join(cwd, "..", "serial-bridge", "serial-bridge.cjs"),
  ];
  const helper = candidates.find((candidate) => {
    try {
      return fs.existsSync(candidate);
    } catch {
      return false;
    }
  });
  if (!helper) return null;
  const localNode = path.join(path.dirname(helper), "node.exe");
  return {
    childProcess,
    helper,
    helperDir: path.dirname(helper),
    nodeExe: fs.existsSync(localNode) ? localNode : "node.exe",
  };
}

function runBridgeList(runtime) {
  return new Promise((resolve, reject) => {
    runtime.childProcess.execFile(
      runtime.nodeExe,
      [runtime.helper, "list"],
      { cwd: runtime.helperDir, windowsHide: true, timeout: 6000 },
      (error, stdout, stderr) => {
        if (error) {
          reject(new Error(stderr?.trim() || error.message));
          return;
        }
        const lines = String(stdout).trim().split(/\r?\n/).filter(Boolean);
        const message = lines.map((line) => {
          try {
            return JSON.parse(line);
          } catch {
            return null;
          }
        }).find((entry) => entry?.event === "ports" || entry?.event === "error");
        if (message?.event === "ports") {
          resolve(message.ports ?? []);
          return;
        }
        reject(new Error(message?.message || "Serial bridge did not return COM ports"));
      },
    );
  });
}

export class UsbSerialTransport extends Transport {
  constructor({ baudRate = 115200 } = {}) {
    super("USB Serial");
    this.baudRate = baudRate;
    this.port = null;
    this.webPorts = [];
    this.webReader = null;
    this.bridge = null;
    this.bridgeRuntime = null;
    this.bridgeBuffer = "";
    this.bridgeOpen = null;
    this.bridgeBootloader = null;
    this.mode = "none";
  }

  async discover() {
    const runtime = bridgeRuntime();
    if (runtime) {
      try {
        const ports = await runBridgeList(runtime);
        if (ports.length > 0) {
          return ports.map((port) => ({
            path: port.path,
            friendlyName: port.friendlyName || port.path,
            manufacturer: port.manufacturer || "",
            serialNumber: port.serialNumber || "",
            vendorId: port.vendorId || "",
            productId: port.productId || "",
            transport: "serial-bridge",
            likelyEsp32: likelyEsp32(port.vendorId, port.productId),
          }));
        }
      } catch (error) {
        this.emitError(error);
      }
    }

    const serial = serialApi();
    if (serial?.getPorts) {
      this.webPorts = await serial.getPorts();
      return [
        ...this.webPorts.map((port, index) => webSerialPortInfo(port, index)),
        {
          path: WEB_SERIAL_PICKER_PATH,
          friendlyName: "Choose USB serial device",
          manufacturer: "Windows Web Serial",
          serialNumber: "",
          vendorId: "",
          productId: "",
          transport: "web-serial",
          likelyEsp32: true,
        },
      ];
    }

    if (runtime) {
      return [];
    }
    throw new Error("COM listing needs the packaged serial bridge, or a runtime with Web Serial enabled.");
  }

  async connect(options) {
    const path = options?.path;
    if (!path) {
      throw new Error("Choose a USB serial device first");
    }
    if (path === WEB_SERIAL_PICKER_PATH || path.startsWith("WEB_SERIAL:")) {
      await this.connectWebSerial(path);
      return;
    }
    await this.connectBridgeSerial(path);
  }

  async connectBridgeSerial(path) {
    const runtime = bridgeRuntime();
    if (!runtime) {
      throw new Error("Serial bridge is not available in this app package");
    }
    this.bridgeRuntime = runtime;
    this.bridgeBuffer = "";
    const bridge = runtime.childProcess.spawn(
      runtime.nodeExe,
      [runtime.helper, "open", "--path", path, "--baud", String(this.baudRate)],
      { cwd: runtime.helperDir, windowsHide: true, stdio: ["pipe", "pipe", "pipe"] },
    );
    this.bridge = bridge;
    this.mode = "serial-bridge";

    const openPromise = new Promise((resolve, reject) => {
      const timer = setTimeout(() => reject(new Error(`Opening ${path} timed out`)), 6000);
      this.bridgeOpen = {
        resolve: () => {
          clearTimeout(timer);
          resolve();
        },
        reject: (error) => {
          clearTimeout(timer);
          reject(error);
        },
      };
    });

    bridge.stdout.on("data", (chunk) => this.handleBridgeStdout(chunk));
    bridge.stderr.on("data", (chunk) => this.emitError(new Error(String(chunk).trim())));
    bridge.on("error", (error) => {
      this.bridgeOpen?.reject(error);
      this.emitError(error);
    });
    bridge.on("exit", (code) => {
      if (!this.connected) return;
      this.setConnected(false, { path, code });
    });

    await openPromise;
    this.setConnected(true, { path, transport: "serial-bridge" });
  }

  handleBridgeStdout(chunk) {
    this.bridgeBuffer += String(chunk);
    for (;;) {
      const lineEnd = this.bridgeBuffer.indexOf("\n");
      if (lineEnd < 0) break;
      const line = this.bridgeBuffer.slice(0, lineEnd).trim();
      this.bridgeBuffer = this.bridgeBuffer.slice(lineEnd + 1);
      if (!line) continue;
      let message;
      try {
        message = JSON.parse(line);
      } catch {
        continue;
      }
      if (message.event === "open") {
        this.bridgeOpen?.resolve();
        this.bridgeOpen = null;
      } else if (message.event === "bootloader") {
        this.bridgeBootloader?.resolve();
        this.bridgeBootloader = null;
      } else if (message.event === "data") {
        this.emitData(Buffer.from(String(message.data || ""), "base64"));
      } else if (message.event === "error") {
        const error = new Error(message.message || "Serial bridge error");
        this.bridgeOpen?.reject(error);
        this.bridgeBootloader?.reject(error);
        this.bridgeBootloader = null;
        this.emitError(error);
      } else if (message.event === "close") {
        this.setConnected(false);
      }
    }
  }

  async connectWebSerial(path) {
    const serial = serialApi();
    if (!serial) {
      throw new Error("Web Serial is not available in this Windows runtime");
    }
    let port = null;
    if (path.startsWith("WEB_SERIAL:")) {
      const index = Number(path.split(":")[1]);
      port = this.webPorts[index] ?? null;
    }
    if (port == null) {
      port = await serial.requestPort();
    }

    await port.open({ baudRate: this.baudRate, bufferSize: 4096 });
    await port.setSignals?.({ dataTerminalReady: true, requestToSend: true });
    this.port = port;
    this.mode = "web-serial";
    this.setConnected(true, { path, transport: "web-serial" });
    this.startWebSerialReadLoop();
  }

  async startWebSerialReadLoop() {
    if (!this.port?.readable || this.webReader) return;
    try {
      while (this.port?.readable && this.connected) {
        this.webReader = this.port.readable.getReader();
        try {
          for (;;) {
            const { value, done } = await this.webReader.read();
            if (done) break;
            if (value?.byteLength > 0) {
              this.emitData(Buffer.from(value));
            }
          }
        } finally {
          this.webReader.releaseLock();
          this.webReader = null;
        }
      }
    } catch (error) {
      if (this.connected) {
        this.emitError(error);
      }
    } finally {
      if (this.connected) {
        this.setConnected(false, { transport: "web-serial" });
      }
    }
  }

  async disconnect() {
    if (this.mode === "serial-bridge") {
      const bridge = this.bridge;
      this.bridge = null;
      this.mode = "none";
      this.bridgeBootloader?.reject(new Error("USB serial bridge disconnected"));
      this.bridgeBootloader = null;
      this.setConnected(false);
      if (bridge && !bridge.killed) {
        bridge.stdin.write(`${JSON.stringify({ op: "close" })}\n`);
        setTimeout(() => {
          if (!bridge.killed) bridge.kill();
        }, 300).unref?.();
      }
      return;
    }

    if (!this.port) {
      this.setConnected(false);
      return;
    }
    const port = this.port;
    this.port = null;
    this.mode = "none";
    this.setConnected(false);
    await this.webReader?.cancel().catch(() => {});
    await port.close().catch(() => {});
  }

  async touchBootloader() {
    if (this.mode === "serial-bridge") {
      if (!this.bridge || this.bridge.killed) {
        throw new Error("USB serial bridge is not connected");
      }
      const promise = new Promise((resolve, reject) => {
        const timer = setTimeout(() => {
          this.bridgeBootloader = null;
          reject(new Error("USB bootloader touch timed out"));
        }, 2500);
        this.bridgeBootloader = {
          resolve: () => {
            clearTimeout(timer);
            resolve();
          },
          reject: (error) => {
            clearTimeout(timer);
            reject(error);
          },
        };
      });
      this.bridge.stdin.write(`${JSON.stringify({ op: "touchBootloader" })}\n`);
      await promise;
      return;
    }

    if (this.mode === "web-serial" && this.port) {
      const port = this.port;
      this.port = null;
      this.mode = "none";
      this.setConnected(false);
      await this.webReader?.cancel().catch(() => {});
      await port.close().catch(() => {});
      await delay(50);
      await port.open({ baudRate: 1200, bufferSize: 4096 });
      await port.setSignals?.({ dataTerminalReady: false, requestToSend: false });
      await delay(120);
      await port.close().catch(() => {});
      return;
    }

    throw new Error("USB bootloader touch needs an open USB serial connection");
  }

  async write(data) {
    if (this.mode === "serial-bridge") {
      if (!this.bridge || this.bridge.killed) {
        throw new Error("USB serial bridge is not connected");
      }
      this.bridge.stdin.write(`${JSON.stringify({
        op: "write",
        data: Buffer.from(data).toString("base64"),
      })}\n`);
      return;
    }

    if (!this.port?.writable) {
      throw new Error("USB serial port is not connected");
    }
    const writer = this.port.writable.getWriter();
    try {
      await writer.write(Buffer.from(data));
    } finally {
      writer.releaseLock();
    }
  }
}
