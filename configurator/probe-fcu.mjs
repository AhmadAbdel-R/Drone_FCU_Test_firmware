// Direct-wire FCU protocol probe + boot capture. Reuses the app's protocol.js
// so framing is byte-identical to the real client.
//   node probe-fcu.mjs [COM3] [115200] [--no-reset]
import { SerialPort } from "serialport";
import {
  encodeFrame,
  ProtocolParser,
  MessageType,
  FrameFlags,
} from "./src/services/protocol.js";

const PATH = (process.argv[2] && !process.argv[2].startsWith("--")) ? process.argv[2] : "COM3";
const BAUD = Number((process.argv[3] && !process.argv[3].startsWith("--")) ? process.argv[3] : "115200");
const NO_RESET = process.argv.includes("--no-reset");
const SOAK_ARG = process.argv.find((a) => a.startsWith("--soak"));
const SOAK = SOAK_ARG != null;
const SOAK_ITERS = SOAK_ARG && SOAK_ARG.includes("=") ? Number(SOAK_ARG.split("=")[1]) : 1500;
const IDLE_ARG = process.argv.find((a) => a.startsWith("--idle"));
const IDLE_S = IDLE_ARG && IDLE_ARG.includes("=") ? Number(IDLE_ARG.split("=")[1]) : 0;
let QUIET = false;
const TYPE_NAMES = Object.fromEntries(Object.entries(MessageType).map(([k, v]) => [v, k]));

const ts = () => new Date().toISOString().slice(11, 23);
const log = (...a) => console.log(ts(), ...a);
const delay = (ms) => new Promise((r) => setTimeout(r, ms));

let parser = new ProtocolParser();
let received = [];
let asciiBuf = "";
let crashHit = false;
let port = null;

function attach(p) {
  p.on("data", (chunk) => {
    const buf = Buffer.from(chunk);
    for (const f of parser.push(buf)) {
      if (!QUIET) {
        const name = TYPE_NAMES[f.type] || "?";
        log(`  FRAME 0x${f.type.toString(16)} (${name}) seq=${f.seq} flags=0x${f.flags.toString(16)} len=${f.payload.length}`);
        if (f.payload.length) log("   payload:", Buffer.from(f.payload).toString("utf8"));
      }
      received.push(f);
    }
    asciiBuf += buf.toString("latin1");
    let nl;
    while ((nl = asciiBuf.indexOf("\n")) >= 0) {
      const line = asciiBuf.slice(0, nl).replace(/\r$/, "");
      asciiBuf = asciiBuf.slice(nl + 1);
      const printable = line.replace(/[^\x20-\x7e]/g, "");
      if (printable.length >= 3 && /[A-Za-z]/.test(printable)) log("TXT|", printable);
      if (/Guru Meditation|Backtrace|abort\(\)|assert fail|rst:0x|panic|Prohibited|ESP-ROM|waiting for download/i.test(line)) {
        crashHit = true;
        log("  *** SERIAL NOTE ***", printable);
      }
    }
  });
  p.on("error", (e) => log("PORT ERROR", e.message));
}

const openPort = () => new Promise((resolve, reject) => {
  const p = new SerialPort({ path: PATH, baudRate: BAUD, autoOpen: false });
  attach(p);
  p.open((e) => (e ? reject(e) : resolve(p)));
});
const setSignals = (p, sig) => new Promise((r) => p.set(sig, () => r()));
const closePort = (p) => new Promise((r) => { if (!p || !p.isOpen) return r(); p.close(() => r()); });
async function listPaths() {
  try { return (await SerialPort.list()).map((x) => x.path); } catch { return []; }
}
const present = async () => (await listPaths()).some((x) => x.toUpperCase() === PATH.toUpperCase());

function waitForSeq(seq, timeoutMs) {
  return new Promise((resolve) => {
    const start = Date.now();
    const iv = setInterval(() => {
      const hit = received.find((f) => f.seq === seq);
      if (hit) { clearInterval(iv); resolve({ ok: true, frame: hit, ms: Date.now() - start }); }
      else if (Date.now() - start > timeoutMs) { clearInterval(iv); resolve({ ok: false, ms: Date.now() - start }); }
    }, 5);
  });
}

let seqCounter = 1;
const summary = [];
async function probe(type, { payload = new Uint8Array(0), timeoutMs = 2500 } = {}) {
  const seq = seqCounter++;
  received.length = 0; // serialized requests: prior responses already matched
  const frame = encodeFrame({ type, seq, flags: FrameFlags.ACK_REQUIRED, payload });
  if (!QUIET) log(`TX  0x${type.toString(16)} (${TYPE_NAMES[type]}) seq=${seq} ${frame.length}B`);
  await new Promise((res, rej) => port.write(frame, (e) => (e ? rej(e) : res())));
  await new Promise((res, rej) => port.drain((e) => (e ? rej(e) : res())));
  const r = await waitForSeq(seq, timeoutMs);
  if (r.ok) {
    if (!QUIET) log(`OK  seq=${seq} -> 0x${r.frame.type.toString(16)} (${TYPE_NAMES[r.frame.type]}) ${r.ms}ms`);
    summary.push(`${TYPE_NAMES[type]}: OK ${r.ms}ms -> ${TYPE_NAMES[r.frame.type]}`);
  } else {
    log(`!!  TIMEOUT seq=${seq} (${TYPE_NAMES[type]}) ${r.ms}ms`);
    summary.push(`${TYPE_NAMES[type]}: TIMEOUT ${r.ms}ms`);
  }
  return r;
}

async function main() {
  log("listing ports:", (await listPaths()).join(", ") || "(none)");
  log("opening", PATH, "@", BAUD);
  port = await openPort();
  log("open ok");

  if (!NO_RESET) {
    log("hard-reset via RTS pulse (DTR low = normal boot)...");
    await setSignals(port, { dtr: false, rts: false });
    await delay(50);
    await setSignals(port, { dtr: false, rts: true });  // EN low (in reset)
    await delay(150);
    await setSignals(port, { dtr: false, rts: false }); // EN high -> normal boot
    await closePort(port);
    log("waiting for native-USB re-enumeration...");
    let gone = false, back = false;
    for (let i = 0; i < 70; i++) {
      const p = await present();
      if (!p) gone = true;
      if (gone && p) { back = true; break; }
      await delay(150);
    }
    log(gone ? (back ? "re-enumerated OK" : "dropped but not back yet") : "port never dropped (RTS reset is a no-op on this board)");
    if (gone && !back) log("current ports:", (await listPaths()).join(", ") || "(none)");
    await delay(500);
    parser = new ProtocolParser(); received = []; asciiBuf = "";
    try {
      port = await openPort();
      log("reopened");
    } catch (e) {
      log("reopen failed:", e.message, "| ports:", (await listPaths()).join(", "));
      process.exit(2);
    }
  }

  await setSignals(port, { dtr: true, rts: true });
  await delay(200);
  log("---- capturing 2.5s of boot/async serial ----");
  await delay(2500);

  if (SOAK) {
    const STYPE_ARG = process.argv.find((a) => a.startsWith("--stype="));
    const stype = STYPE_ARG ? STYPE_ARG.split("=")[1] : "mix";
    const typeMap = {
      adv: [MessageType.GET_ADVANCED_STATUS],
      state: [MessageType.GET_STATE],
      sensor: [MessageType.GET_SENSOR_STATUS],
      caps: [MessageType.GET_CAPABILITIES],
      mix: [MessageType.GET_STATE, MessageType.GET_ADVANCED_STATUS],
    };
    const types = typeMap[stype] || typeMap.mix;
    log(`---- SOAK[${stype}]: hammering ${types.map((t) => TYPE_NAMES[t]).join("/")} up to ${SOAK_ITERS} iters ----`);
    QUIET = true;
    let ok = 0, i = 0, stalls = 0;
    let frozen = false;
    for (i = 1; i <= SOAK_ITERS; i++) {
      const t = types[i % types.length];
      const r = await probe(t, { timeoutMs: 1500 });
      if (r.ok) { ok++; }
      else {
        QUIET = false;
        stalls++;
        log(`>>> iter ${i}: ${TYPE_NAMES[t]} stall #${stalls} (ok=${ok}); checking liveness up to ~6s...`);
        let revived = false;
        for (let a = 0; a < 5 && !revived; a++) {
          const h = await probe(MessageType.HELLO, { timeoutMs: 3000 });
          if (h.ok) revived = true;
        }
        if (!revived) {
          log(`>>> PERMANENT FREEZE at iter ${i} — HELLO dead ~6s. Firmware wedged.`);
          frozen = true;
          break;
        }
        log(`>>> recovered at iter ${i} (transient stall); continuing`);
        QUIET = true;
      }
      if (i % 100 === 0) log(`...soak ${i}/${SOAK_ITERS} ok=${ok} stalls=${stalls}`);
      summary.length = 0; // don't accumulate thousands of summary lines
    }
    QUIET = false;
    const done = i > SOAK_ITERS ? SOAK_ITERS : i;
    log(`===== SOAK DONE: ${done} iters, ${ok} ok, ${stalls} transient stall(s), frozen=${frozen} =====`);
    await closePort(port);
    process.exit(0);
  }

  if (IDLE_S > 0) {
    log(`---- IDLE ${IDLE_S}s (no requests) — testing time-based freeze ----`);
    for (let s = 0; s < IDLE_S; s++) { await delay(1000); if ((s + 1) % 5 === 0) log(`...idled ${s + 1}/${IDLE_S}s`); }
    log("---- idle done; now one probe burst to check if firmware is alive ----");
  }

  log("---- differential probe ----");
  await probe(MessageType.HELLO);
  await delay(60);
  await probe(MessageType.GET_CAPABILITIES);
  await delay(60);
  await probe(MessageType.GET_ADVANCED_STATUS); // uses health/servo/notch — NOT getDashTelemetry
  await delay(60);
  await probe(MessageType.GET_STATE);           // uses getDashTelemetry  <-- prime suspect
  await delay(60);
  await probe(MessageType.GET_CONFIG, { payload: Uint8Array.from([1]) }); // canary after STATE
  await delay(60);
  await probe(MessageType.GET_SENSOR_STATUS);   // also uses getDashTelemetry

  log("===== SUMMARY =====");
  for (const s of summary) log("  " + s);
  await closePort(port);
  process.exit(0);
}

main().catch((e) => { log("FATAL", e.message); process.exit(1); });
