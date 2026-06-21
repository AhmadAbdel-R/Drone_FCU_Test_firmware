#!/usr/bin/env node
"use strict";

const { SerialPort } = require("serialport");

function send(message) {
  process.stdout.write(`${JSON.stringify(message)}\n`);
}

function fail(message, detail = "") {
  send({ event: "error", message, detail });
  process.exitCode = 1;
}

function argValue(name, fallback = "") {
  const index = process.argv.indexOf(name);
  return index >= 0 && index + 1 < process.argv.length ? process.argv[index + 1] : fallback;
}

function delay(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

function callPortMethod(port, method, ...args) {
  if (typeof port[method] !== "function") return Promise.resolve();
  return new Promise((resolve) => {
    port[method](...args, () => resolve());
  });
}

async function listPorts() {
  const ports = await SerialPort.list();
  send({
    event: "ports",
    ports: ports.map((port) => ({
      path: port.path,
      friendlyName: port.friendlyName || port.path,
      manufacturer: port.manufacturer || "",
      serialNumber: port.serialNumber || "",
      vendorId: port.vendorId || "",
      productId: port.productId || "",
      pnpId: port.pnpId || "",
    })),
  });
}

async function openPort() {
  const path = argValue("--path");
  const baudRate = Number(argValue("--baud", "115200"));
  if (!path) {
    fail("Missing --path");
    return;
  }
  if (!Number.isInteger(baudRate) || baudRate <= 0) {
    fail("Invalid --baud");
    return;
  }

  const port = new SerialPort({ path, baudRate, autoOpen: false });
  let closed = false;

  port.on("data", (chunk) => {
    send({ event: "data", data: Buffer.from(chunk).toString("base64") });
  });
  port.on("error", (error) => {
    send({ event: "error", message: error.message || String(error) });
  });
  port.on("close", () => {
    closed = true;
    send({ event: "close" });
    process.exit(0);
  });

  await new Promise((resolve, reject) => {
    port.open((error) => (error ? reject(error) : resolve()));
  });
  await callPortMethod(port, "set", { dtr: true, rts: true });
  await delay(350);
  send({ event: "open", path });

  let pending = "";
  process.stdin.setEncoding("utf8");
  process.stdin.on("data", (chunk) => {
    pending += chunk;
    for (;;) {
      const lineEnd = pending.indexOf("\n");
      if (lineEnd < 0) break;
      const line = pending.slice(0, lineEnd).trim();
      pending = pending.slice(lineEnd + 1);
      if (!line) continue;
      let command;
      try {
        command = JSON.parse(line);
      } catch {
        send({ event: "error", message: "Bad command JSON" });
        continue;
      }
      if (command.op === "close") {
        if (!closed) port.close(() => {});
        continue;
      }
      if (command.op === "touchBootloader") {
        port.update({ baudRate: 1200 }, async (updateError) => {
          if (updateError) {
            send({ event: "error", message: updateError.message || String(updateError) });
            return;
          }
          await callPortMethod(port, "set", { dtr: false, rts: false });
          await delay(120);
          send({ event: "bootloader" });
          if (!closed) port.close(() => {});
        });
        continue;
      }
      if (command.op === "write") {
        const data = Buffer.from(String(command.data || ""), "base64");
        port.write(data, (writeError) => {
          if (writeError) {
            send({ event: "error", message: writeError.message || String(writeError) });
            return;
          }
          port.drain((drainError) => {
            if (drainError) {
              send({ event: "error", message: drainError.message || String(drainError) });
            }
          });
        });
      }
    }
  });
  process.stdin.on("end", () => {
    if (!closed) port.close(() => {});
  });
}

async function main() {
  const command = process.argv[2];
  if (command === "list") {
    await listPorts();
  } else if (command === "open") {
    await openPort();
  } else {
    fail("Expected command: list or open");
  }
}

main().catch((error) => {
  fail(error.message || String(error), error.stack || "");
});
