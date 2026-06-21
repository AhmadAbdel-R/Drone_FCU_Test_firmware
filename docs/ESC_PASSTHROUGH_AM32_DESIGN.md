# ESC Passthrough over USB — AM32 feasibility study & design

**ESC:** Sequre Blueson A2 65A (4-in-1), firmware **AM32**
**FCU:** ESP32-S3-MINI, native USB-CDC console, motors M1–M4 on GPIO39–42 (DShot300, one-way)
**Date:** 2026-06-14
**Status:** investigation + design. The firmware currently ships only the **safe
scaffold + dry-run** (`include/esc_usb_passthrough.h`, `src/esc_usb_passthrough.cpp`).
The real protocol described below is **not implemented yet**.

---

## 1. What "configure the ESC over USB" actually requires

There is **no UART configuration port** on a DShot 4-in-1. AM32 (like BLHeli_S /
BLHeli_32) is configured *through the flight controller* using a two-layer
protocol over the FC's USB serial link:

```
  AM32 configurator / esc-configurator.com   (PC, Web Serial)
        │  layer A: MSP  (detect the FC, then request passthrough)
        ▼
  FCU USB-CDC  ──►  MSP responder  ──►  4-way interface state machine
        │                                      │ layer B: BLHeli/AM32
        │                                      ▼  single-wire bootloader
        └────────────────────────────►  bit-bang on ONE motor signal pin
                                               ▼
                                        AM32 ESC bootloader
```

**Layer A — MSP.** The configurator first speaks [MSP](https://github.com/iNavFlight/inav/wiki/MSP-V2)
to identify the board, then sends `MSP_SET_PASSTHROUGH` (command **245**) with
payload byte `MSP_PASSTHROUGH_ESC_4WAY = 0xFF` to switch the port into 4-way
mode. (Confirmed in the vendored Betaflight reference at
`betaflight/src/main/msp/msp.c:204,303` and `msp.c:323` → `esc4wayProcess`.)

Minimum MSP the configurator needs to detect the FC and proceed:

| MSP cmd | # | Purpose |
|---|---|---|
| `MSP_API_VERSION` | 1 | protocol/version handshake (must answer first) |
| `MSP_FC_VARIANT` | 2 | 4-char FC identifier (e.g. `"BTFL"`) |
| `MSP_FC_VERSION` | 3 | FC semantic version |
| `MSP_BOARD_INFO` | 4 | board id |
| `MSP_BUILD_INFO` | 5 | build string |
| `MSP_UID` | 160 | unique id (some clients) |
| `MSP_MOTOR` | 104 | motor count / which outputs exist |
| `MSP_SET_PASSTHROUGH` | 245 | enter 4-way (payload `0xFF`) |

**Layer B — the 4-way interface + bootloader.** Once in 4-way mode the byte
stream is no longer MSP; it is the BLHeli "4-way interface" by *4712*
(`betaflight/src/main/io/serial_4way.{c,h}`). It frames commands
(`cmd_DeviceInitFlash`, `cmd_DeviceRead`, `cmd_DeviceWrite`, `cmd_DevicePageErase`,
`cmd_DeviceReset`, …) with a CRC16, and **selects which ESC** (1 of 4) to talk
to. For each command it drives that ESC's bootloader on its **signal wire**.

The interface supports several device families via an *interface mode*:
`imSIL_BLB`(1)=BLHeli_S SiLabs, `imATM_BLB`(2)=Atmel, `imSK`(3)=SimonK,
**`imARM_BLB`(4)=ARM** (`serial_4way.h:29-33`). **AM32 is `imARM_BLB`** — it
implements a bootloader compatible with the same BLHeli-bootloader wire protocol
(`serial_4way.c` lines 354/368/601/669-673 route ARM through the same
`serial_4way_avrootloader.c` `BL_*` functions).

### The wire protocol (from `serial_4way_avrootloader.c`)

- **Single-wire, half-duplex, software-UART** on the motor signal pin.
- **`BIT_TIME = 52 µs` ≈ 19.2 kbit/s**, 1 start bit + 8 data + 1 stop
  (`suart_putc_` / `suart_getc_`).
- **Connect:** send `{0,0,0,0,0,0,0,0,0x0D,'B','L','H','e','l','i',0xF4,0x7D}`
  (17 bytes), read 8-byte boot info, verify it starts with `"471"`, capture the
  device signature.
- **CRC16**, poly `0xA001` (reflected CCITT/IBM), appended once the MCU is
  connected.
- **Commands:** `SET_ADDRESS 0xFF`, `SET_BUFFER 0xFE`, `READ_FLASH_SIL 0x03`,
  `PROG_FLASH 0x01`, `ERASE_FLASH 0x02`, `READ_EEPROM 0x04`, `PROG_EEPROM 0x05`,
  `KEEP_ALIVE 0xFD`, ARM verify `0x04`.
- The **EEPROM page (address 0x7C00 on many AM32 targets)** holds the settings
  the configurator reads/writes; flashing firmware writes the flash pages.

This is the *whole* job. Nothing about it is fake or approximate — it's a known,
open protocol.

---

## 2. Can the ESP32-S3 do this over its USB-CDC port? — Yes, with one hard part

| Layer | Verdict | Why |
|---|---|---|
| USB-CDC transport | ✅ easy | Native USB-CDC is **baud-independent** — the configurator opens the COM port at any baud and it just works. MSP + 4-way are plain byte streams. |
| MSP responder | ✅ moderate | ~8 read commands + `MSP_SET_PASSTHROUGH`. A few hundred lines; no timing constraints. |
| 4-way state machine | ✅ moderate | Framing + CRC16 + per-ESC dispatch. Self-contained logic, no timing constraints. |
| Single-wire bootloader bit-bang | ⚠️ **the hard part** | Software-UART at 52 µs/bit on the GPIO. Needs **jitter-free microsecond timing** that FreeRTOS context switches, the WiFi stack, and other ISRs will corrupt mid-byte. |

### Why the bit-bang is the risk — and why it's still feasible here

The reference `suart_getc_`/`suart_putc_` busy-wait on `micros()` between bit
edges. A single preemption during a byte desynchronises the frame. **Betaflight's
own ESP32-S3 port gives up on this** — it explicitly disables the whole feature:

```
betaflight/src/platform/ESP32/target/ESP32S3/target.h:113-115
  #undef USE_SERIAL_4WAY_BLHELI_INTERFACE
  #undef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
  #undef USE_SERIAL_4WAY_SK_BOOTLOADER
```

That is the real-world signal that this is non-trivial — **not** that it's
impossible. It's feasible on *this* firmware precisely because passthrough is a
**bench-only mode** we fully control:

- It only runs **disarmed, motors stopped, DShot suspended** (the scaffold
  already guarantees this).
- The flight task **backs off the motors** while active (it already does).
- So we can legitimately **disable interrupts for the duration of each byte**
  (`portDISABLE_INTERRUPTS()` / a critical section, or pin the bit-bang to the
  otherwise-idle core and suspend the other tasks). 17–256 byte transactions at
  52 µs/bit are short (≤ ~13 ms worst case), and there is nothing flight-critical
  to starve while the props are off.
- WiFi/BLE are already off in this image, removing the worst jitter source.

**Conclusion: feasible.** The MSP + 4-way layers are straightforward; the
bootloader bit-bang is achievable by bracketing each byte in an interrupts-off
critical section, which is acceptable only because of the safe, quiesced state
the scaffold enforces. Expect the bit-bang timing to need oscilloscope/real-ESC
validation.

---

## 3. Why the vendored library's passthrough can't be used as-is

`lib/easy-esc-esp32/src/esc_passthrough.cpp` is a **placeholder** (its own header
says *"BLHeli_S command sequencing is still incomplete"*). It bit-bangs an
**invented** framing (additive checksum, made-up start byte) and a *"simplified
placeholder"* handshake that match **no** real bootloader. Its **pin handoff is
real and reused** by the scaffold (`suspendMotorDriverForPassthrough` /
`resumeMotorDriverFromPassthrough` genuinely tear down and rebuild the RMT
driver), but its `passthroughRead/Write/Erase` must **not** be called — they
would be fake protocol. The real implementation replaces that wire layer
entirely.

---

## 4. Licensing — important

- **Betaflight is GPLv3.** The vendored `betaflight/` tree is a *reference only*
  (untracked in git). Copying `serial_4way*.c` into this firmware would impose
  GPLv3 on the whole image. **Do not copy it.** Protocol facts (byte values,
  timings, CRC) are not copyrightable and may be reimplemented.
- **AM32's bootloader is MIT-licensed** (`AlkaMotors/AM32-MultiRotor-ESC-firmware`,
  `bootloader/`). That is the clean reference to port the wire layer from, and it
  documents the exact AM32 boot signature/EEPROM layout for this ESC.
- Write **original** MSP/4-way/bootloader code; cite both as references.

---

## 5. Proposed design (next step — not yet built)

Keep the safe scaffold; add a real bridge behind the same `isActive()` gate.

```
src/esc_usb_passthrough.cpp         (existing) safety + lifecycle + CLI host
  └─ when ENABLE_ESC_PASSTHROUGH_BRIDGE=1, hand the live byte stream to:
include/esc_msp_bridge.h / .cpp     NEW  minimal MSP responder + MSP_SET_PASSTHROUGH
include/esc_4way.h / .cpp           NEW  4-way framing + CRC16 + per-ESC dispatch
include/esc_arm_bootloader.h / .cpp NEW  imARM_BLB single-wire bit-bang (AM32), IRQ-off
```

Flow:
1. `esc passthrough on` → existing scaffold disarms, stops motors, suspends DShot
   on all four pins (unchanged).
2. `pollCli` (or a dedicated bench task) feeds USB bytes to the **MSP responder**
   instead of the dry-run pump.
3. On `MSP_SET_PASSTHROUGH 0xFF`, switch the parser to the **4-way** state machine.
4. 4-way `DeviceInitFlash(motor, imARM_BLB)` → select that motor's GPIO →
   `BL_ConnectEx` on the wire → subsequent read/write/erase mapped to `BL_*`.
5. Each wire byte runs inside `portENTER_CRITICAL`/IRQ-off; the 4-way layer in
   between is plain logic.
6. 4-way `DeviceReset` / exit, or the existing inactivity timeout → scaffold
   restores DShot (unchanged). Motors never auto-arm.

### Open items to verify on the bench
- [ ] Native USB-CDC vs. the configurator's Web Serial baud/flush expectations.
- [ ] Minimal MSP set that makes esc-configurator show "connected" + the 4 ESCs.
- [ ] AM32 boot signature + EEPROM base for the Blueson A2's MCU (read MIT bootloader).
- [ ] Bit-bang timing on GPIO39–42 with IRQ-off — scope the 52 µs bit cell.
- [ ] Full read→edit→write→verify round-trip against the real ESC (props off).
- [ ] USB logging must be **fully** muted in 4-way mode (no log bytes in the stream).

### Effort / risk
- MSP + 4-way: ~1–2 days, low risk (pure logic, testable off-target).
- ARM bootloader bit-bang: ~1–3 days + bench time, **medium risk** (timing).
- Recommend implementing read-only first (read settings) before enabling writes.

---

## 6. References
- Vendored reference (GPLv3, study-only): `betaflight/src/main/io/serial_4way.c`,
  `serial_4way_avrootloader.c`, `betaflight/src/main/msp/msp.c`,
  `betaflight/src/platform/ESP32/target/ESP32S3/target.h`.
- AM32 firmware + bootloader (MIT): `github.com/AlkaMotors/AM32-MultiRotor-ESC-firmware`.
- esc-configurator (web): `github.com/stylesuxx/esc-configurator`.
