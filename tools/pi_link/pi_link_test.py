#!/usr/bin/env python3
"""FCU <-> Pi UART link smoke test, runs on the Raspberry Pi Zero 2W.

Wiring (3.3 V, common GND):
    Pi GPIO14 (TXD0, pin 8)   ->  FCU RX  (GPIO 18)
    Pi GPIO15 (RXD0, pin 10)  <-  FCU TX  (GPIO 17)
    Pi GND                    --  FCU GND

On the Pi:
    sudo raspi-config -> Interface Options -> Serial Port
        - Login shell over serial: NO
        - Serial port hardware enabled: YES
    Reboot, then /dev/serial0 maps to the GPIO UART.

Run on the Pi:
    pip install pyserial
    python3 pi_link_test.py

What this script does:
    - Sends PI_HEARTBEAT,<count>,<unix_ms> once per second
    - Reads any line from the FCU (GPS / FCU_STATE) and prints it
    - Optional: hit a key (h, f, b, l, r, yl, yr, land, stop) to send an
      autonomy command (PI_CMD,<token>)

Press Ctrl-C to exit.
"""
from __future__ import annotations

import argparse
import select
import sys
import time
from contextlib import suppress

import serial


CMD_KEYS = {
    "h": "HOVER",
    "f": "FORWARD",
    "b": "BACKWARD",
    "l": "LEFT",
    "r": "RIGHT",
    "yl": "YAW_LEFT",
    "yr": "YAW_RIGHT",
    "land": "LAND",
    "stop": "STOP",
}


def now_ms() -> int:
    return int(time.time() * 1000)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/serial0", help="serial device")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--heartbeat-hz",
        type=float,
        default=1.0,
        help="heartbeat rate (FCU expects ~1 Hz, link timeout is 2.5 s)",
    )
    parser.add_argument(
        "--no-input",
        action="store_true",
        help="don't accept interactive commands (useful when piping)",
    )
    args = parser.parse_args()

    print(f"Opening {args.port} @ {args.baud} ...")
    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    print("Opened. Sending heartbeats. Lines from FCU will appear below.")
    if not args.no_input:
        print(
            "Command keys (type then Enter): "
            "h=HOVER, f=FORWARD, b=BACKWARD, l=LEFT, r=RIGHT, "
            "yl=YAW_LEFT, yr=YAW_RIGHT, land=LAND, stop=STOP, q=quit"
        )
    print("-" * 60)

    hb_count = 0
    hb_period = 1.0 / max(0.1, args.heartbeat_hz)
    next_hb = time.monotonic()
    line_buf = b""

    def send_cmd(token: str) -> None:
        payload = f"PI_CMD,{token}\n".encode()
        try:
            ser.write(payload)
        except Exception as e:
            print(f"  ! write failed: {e}", file=sys.stderr)

    def send_heartbeat() -> None:
        nonlocal hb_count
        hb_count += 1
        payload = f"PI_HEARTBEAT,{hb_count},{now_ms()}\n".encode()
        try:
            ser.write(payload)
        except Exception as e:
            print(f"  ! heartbeat write failed: {e}", file=sys.stderr)

    try:
        while True:
            t = time.monotonic()
            if t >= next_hb:
                send_heartbeat()
                next_hb += hb_period
                # Resync if we fell badly behind (e.g. system was sleeping).
                if t > next_hb + 5 * hb_period:
                    next_hb = t + hb_period

            # Drain incoming bytes, split into lines.
            data = ser.read(256)
            if data:
                line_buf += data
                while b"\n" in line_buf:
                    line, line_buf = line_buf.split(b"\n", 1)
                    text = line.decode(errors="replace").rstrip("\r")
                    if text:
                        print(f"<- {text}")

            # Non-blocking stdin (for ad-hoc commands). select.select with a
            # 0 timeout returns immediately.
            if not args.no_input and select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.readline().strip().lower()
                if key in ("q", "quit", "exit"):
                    break
                token = CMD_KEYS.get(key)
                if token:
                    print(f"-> PI_CMD,{token}")
                    send_cmd(token)
                elif key:
                    print(f"  ? unknown key '{key}'")

            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        # Best-effort: send a STOP on the way out so the FCU returns to a
        # safe state if autonomy was active.
        with suppress(Exception):
            ser.write(b"PI_CMD,STOP\n")
            ser.flush()
        ser.close()
        print("Closed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
