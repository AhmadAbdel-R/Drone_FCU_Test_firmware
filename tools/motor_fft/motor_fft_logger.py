#!/usr/bin/env python3
"""Motor FFT logger and analyzer for the fcu_motor_fft_test firmware.

Reads the CSV gyro/accel stream emitted by the test firmware over USB serial,
saves it to disk, then runs an FFT pass and prints suggested notch-filter
centre frequencies for the dominant vibration peaks.

Typical usage:

    python motor_fft_logger.py --port COM7

To re-analyze an existing CSV without connecting to the drone:

    python motor_fft_logger.py --analyze motor_fft_20260518_142211.csv

SAFETY: this tool drives motors. Remove props for motor mapping.
"""
from __future__ import annotations

import argparse
import csv
import os
import queue
import sys
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
import serial
from scipy.fft import rfft, rfftfreq
from scipy.signal import find_peaks, spectrogram
import matplotlib

# Use a non-blocking backend so plt.show() doesn't freeze the CLI.
matplotlib.use("TkAgg") if "TkAgg" in matplotlib.rcsetup.all_backends else None
import matplotlib.pyplot as plt  # noqa: E402


CSV_HEADER = [
    "timestamp_us",
    "gyro_x",
    "gyro_y",
    "gyro_z",
    "accel_x",
    "accel_y",
    "accel_z",
    "motor_id",
    "motor_command",
]


@dataclass
class Sample:
    ts_us: int
    gx: float
    gy: float
    gz: float
    ax: float
    ay: float
    az: float
    motor_id: int
    motor_cmd: int


# =============================================================================
# Serial reader thread
# =============================================================================
def reader_loop(ser: serial.Serial, line_q: queue.Queue, stop_evt: threading.Event):
    while not stop_evt.is_set():
        try:
            raw = ser.readline()
        except serial.SerialException as exc:
            line_q.put(f"[ERROR] serial read failed: {exc}")
            break
        if not raw:
            continue
        try:
            line = raw.decode(errors="replace").rstrip("\r\n")
        except Exception:
            continue
        if line:
            line_q.put(line)


# =============================================================================
# Session: owns the serial port, log file, and sample buffer
# =============================================================================
@dataclass
class LogSession:
    csv_path: Path
    writer: csv.writer
    file_handle: "open"
    samples: List[Sample] = field(default_factory=list)


class Logger:
    def __init__(self, ser: serial.Serial):
        self.ser = ser
        self.line_q: queue.Queue = queue.Queue()
        self.stop_evt = threading.Event()
        self.reader = threading.Thread(
            target=reader_loop, args=(self.ser, self.line_q, self.stop_evt), daemon=True
        )
        self.reader.start()
        self.session: Optional[LogSession] = None
        # Default to a "passive printer" thread that prints non-CSV lines as
        # they arrive, so the user sees firmware status without typing.
        self.printer_stop = threading.Event()
        self.printer = threading.Thread(target=self._printer_loop, daemon=True)
        self.printer.start()

    # ---------------------------------------------------------------------
    def send(self, cmd: str) -> None:
        if not cmd.endswith("\n"):
            cmd += "\n"
        self.ser.write(cmd.encode())

    def start_log(self, name: Optional[str] = None) -> None:
        if self.session is not None:
            print("(already logging — sending STOP first if you want to restart)")
            return
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = Path(name) if name else Path(f"motor_fft_{ts}.csv")
        fh = open(path, "w", newline="")
        w = csv.writer(fh)
        w.writerow(CSV_HEADER)
        self.session = LogSession(csv_path=path, writer=w, file_handle=fh)
        self.send("LOG_START")
        print(f"Logging to {path}. Use 'log stop' to finalize.")

    def stop_log(self) -> None:
        if self.session is None:
            print("(not logging)")
            return
        self.send("LOG_STOP")
        # Let the firmware drain the ring buffer and reply.
        time.sleep(0.7)
        self.session.file_handle.flush()
        self.session.file_handle.close()
        path = self.session.csv_path
        samples = self.session.samples
        self.session = None
        print(f"Saved {len(samples)} samples to {path}")
        analyze_samples(samples, path)

    def run_sweep(self, dshot: int, seconds: float = 5.0) -> None:
        """Run the full battery: motor 1, 2, 3, 4 each then all 4 together.
        Each phase writes its own CSV/PNG/report. Final summary prints the
        top peak per axis per phase so you can spot a bad motor or the
        cumulative spectrum used for notch-filter tuning.
        """
        if self.session is not None:
            print("(stop the current log before running sweep)")
            return
        if dshot != 0 and (dshot < 48 or dshot > 2047):
            print(f"refused: dshot {dshot} must be 0 or 48..2047")
            return

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        phases = [
            ("motor1", f"TEST_MOTOR 1 {dshot}"),
            ("motor2", f"TEST_MOTOR 2 {dshot}"),
            ("motor3", f"TEST_MOTOR 3 {dshot}"),
            ("motor4", f"TEST_MOTOR 4 {dshot}"),
            ("all4",   f"TEST_ALL {dshot}"),
        ]
        all_results: List[Tuple[str, Path, List[Tuple[str, List[Tuple[float, float]]]]]] = []

        try:
            for label, cmd in phases:
                print(f"\n=== sweep phase: {label} (DShot {dshot}, {seconds:.1f}s) ===")
                self.send(cmd)
                time.sleep(1.0)  # let the ramp finish climbing to target

                csv_path = Path(f"sweep_{label}_{ts}.csv")
                fh = open(csv_path, "w", newline="")
                w = csv.writer(fh)
                w.writerow(CSV_HEADER)
                self.session = LogSession(csv_path=csv_path, writer=w, file_handle=fh)
                self.send("LOG_START")
                time.sleep(seconds)
                self.send("LOG_STOP")
                time.sleep(0.7)  # drain the firmware ring

                phase_samples = self.session.samples[:]
                self.session.file_handle.flush()
                self.session.file_handle.close()
                self.session = None
                self.send("STOP")
                time.sleep(0.5)

                print(f"  captured {len(phase_samples)} samples -> {csv_path}")
                peaks = analyze_samples(phase_samples, csv_path, show_plot=False)
                all_results.append((label, csv_path, peaks))
        except KeyboardInterrupt:
            print("\nsweep interrupted")
        finally:
            self.send("STOP")
            if self.session is not None:
                self.session.file_handle.close()
                self.session = None

        # Final cross-phase summary.
        print("\n========== SWEEP SUMMARY ==========")
        print(f"throttle DShot {dshot}, {seconds:.1f}s per phase")
        for label, path, peaks in all_results:
            top = []
            for axis, axis_peaks in peaks:
                if axis_peaks:
                    f, mag = axis_peaks[0]
                    top.append(f"{axis} {f:.0f}Hz/{mag:.0f}")
            print(f"  {label:6s} top1: {' | '.join(top) if top else '(no peaks)'}")
        print(f"PNG + report files saved alongside each sweep_*_{ts}.csv")
        print("===================================")

    def run_ramp(self, dshot_start: int, dshot_end: int, duration_sec: float = 10.0,
                 motor_id: int = 0) -> None:
        """Throttle ramp capture for spectrogram analysis.

        motor_id = 0 means TEST_ALL (all four motors together).
        motor_id = 1..4 means TEST_MOTOR <id>.

        The firmware enforces DShot >= 48 for spinning, so start values below
        that get bumped up. A reasonable default ramp is 48 -> 2000 over 10 s.
        """
        if self.session is not None:
            print("(stop the current log before running ramp)")
            return
        # Clamp.
        dshot_start = max(0, min(2047, int(dshot_start)))
        dshot_end = max(0, min(2047, int(dshot_end)))
        if dshot_start != 0 and dshot_start < 48:
            print(f"start {dshot_start} < min spin DShot 48; using 48")
            dshot_start = 48
        if dshot_end != 0 and dshot_end < 48:
            print(f"end {dshot_end} < min spin DShot 48; using 48")
            dshot_end = 48
        if motor_id < 0 or motor_id > 4:
            print("motor_id must be 0 (all) or 1..4")
            return

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        label = "all4" if motor_id == 0 else f"motor{motor_id}"
        csv_path = Path(f"ramp_{label}_{ts}.csv")
        cmd_prefix = "TEST_ALL " if motor_id == 0 else f"TEST_MOTOR {motor_id} "

        print(f"\n=== ramp: {label} DShot {dshot_start} -> {dshot_end} over {duration_sec:.1f}s ===")
        # Send start value first so the firmware ramps up to it before logging.
        self.send(cmd_prefix + str(dshot_start))
        time.sleep(1.0)

        # Open log session.
        fh = open(csv_path, "w", newline="")
        w = csv.writer(fh)
        w.writerow(CSV_HEADER)
        self.session = LogSession(csv_path=csv_path, writer=w, file_handle=fh)
        self.send("LOG_START")

        try:
            # Step the throttle ~20 Hz to give the spectrogram a smooth ramp.
            n_steps = max(20, int(duration_sec * 20))
            step_dt = duration_sec / n_steps
            t_start = time.time()
            for i in range(n_steps + 1):
                frac = i / n_steps
                cmd_value = int(round(dshot_start + (dshot_end - dshot_start) * frac))
                self.send(cmd_prefix + str(cmd_value))
                time.sleep(max(0.0, step_dt - (time.time() - t_start - i * step_dt)))
        except KeyboardInterrupt:
            print("\nramp interrupted")
        finally:
            self.send("LOG_STOP")
            time.sleep(0.7)
            phase_samples = self.session.samples[:]
            self.session.file_handle.flush()
            self.session.file_handle.close()
            self.session = None
            self.send("STOP")
            time.sleep(0.3)

        print(f"  captured {len(phase_samples)} samples -> {csv_path}")
        analyze_ramp_samples(phase_samples, csv_path, show_plot=True)

    def replay_csv(self, path: Path) -> None:
        """Re-analyze a previously saved CSV without serial."""
        if not path.exists():
            print(f"file not found: {path}")
            return
        samples = load_csv_samples(path)
        analyze_samples(samples, path, show_plot=True)

    def close(self) -> None:
        try:
            self.send("STOP")
        except Exception:
            pass
        time.sleep(0.1)
        self.stop_evt.set()
        self.printer_stop.set()
        if self.session is not None:
            self.session.file_handle.close()
            self.session = None
        try:
            self.ser.close()
        except Exception:
            pass

    # ---------------------------------------------------------------------
    def _printer_loop(self) -> None:
        while not self.printer_stop.is_set():
            try:
                line = self.line_q.get(timeout=0.1)
            except queue.Empty:
                continue
            self._handle_line(line)

    def _handle_line(self, line: str) -> None:
        if line.startswith("[CSV] ") and self.session is not None:
            payload = line[len("[CSV] "):]
            parts = payload.split(",")
            if len(parts) != 9:
                return
            try:
                sample = Sample(
                    ts_us=int(parts[0]),
                    gx=float(parts[1]),
                    gy=float(parts[2]),
                    gz=float(parts[3]),
                    ax=float(parts[4]),
                    ay=float(parts[5]),
                    az=float(parts[6]),
                    motor_id=int(parts[7]),
                    motor_cmd=int(parts[8]),
                )
            except ValueError:
                return
            self.session.writer.writerow(parts)
            self.session.samples.append(sample)
        elif line.startswith("[CSV] "):
            # Logging not active — drop silently to avoid console spam.
            return
        else:
            print(line)


# =============================================================================
# FFT analysis
# =============================================================================
def _axis_fft(values: np.ndarray, dt_s: float) -> Tuple[np.ndarray, np.ndarray]:
    """Return (freqs_hz, magnitude) for a single-axis time series."""
    values = values - np.mean(values)
    window = np.hanning(len(values))
    spectrum = np.abs(rfft(values * window))
    freqs = rfftfreq(len(values), d=dt_s)
    return freqs, spectrum


def _top_peaks(
    freqs: np.ndarray, spectrum: np.ndarray, max_peaks: int = 5
) -> List[Tuple[float, float]]:
    if len(spectrum) < 8:
        return []
    height = float(np.max(spectrum) * 0.10)
    peak_idx, _ = find_peaks(spectrum, height=height, distance=4)
    order = np.argsort(spectrum[peak_idx])[::-1][:max_peaks]
    return [(float(freqs[peak_idx[i]]), float(spectrum[peak_idx[i]])) for i in order]


def _cluster_peaks(all_peaks: List[float], min_gap_hz: float = 6.0) -> List[float]:
    if not all_peaks:
        return []
    all_peaks = sorted(all_peaks)
    out = [all_peaks[0]]
    for f in all_peaks[1:]:
        if abs(f - out[-1]) > min_gap_hz:
            out.append(f)
    return out


def analyze_samples(samples: List[Sample], source_path: Path, *, show_plot: bool = True) -> List[Tuple[str, List[Tuple[float, float]]]]:
    if len(samples) < 256:
        print(f"Only {len(samples)} samples captured — too few for a useful FFT.")
        return []

    ts = np.array([s.ts_us for s in samples], dtype=np.float64)
    gx = np.array([s.gx for s in samples])
    gy = np.array([s.gy for s in samples])
    gz = np.array([s.gz for s in samples])
    motor_ids = np.array([s.motor_id for s in samples])
    motor_cmds = np.array([s.motor_cmd for s in samples])

    dt_us = np.diff(ts)
    valid = (dt_us > 0) & (dt_us < 100000)
    if not valid.any():
        print("ERROR: timestamps are not monotonic — cannot estimate sample rate.")
        return
    median_dt_us = float(np.median(dt_us[valid]))
    sample_rate = 1.0e6 / median_dt_us
    dt_s = median_dt_us * 1.0e-6

    active = motor_ids[motor_ids > 0]
    motor_under_test = int(active[len(active) // 2]) if len(active) > 0 else 0
    active_cmd = motor_cmds[motor_ids > 0]
    cmd_under_test = int(active_cmd[len(active_cmd) // 2]) if len(active_cmd) > 0 else 0

    print()
    print("====================== FFT ANALYSIS ======================")
    print(f"Source           : {source_path}")
    print(f"Samples          : {len(samples)}")
    print(f"Sample rate (est): {sample_rate:.1f} Hz")
    print(f"Motor tested     : {motor_under_test} @ DShot {cmd_under_test}")

    peaks_per_axis: List[Tuple[str, List[Tuple[float, float]]]] = []
    spectra: List[Tuple[str, np.ndarray, np.ndarray]] = []
    for label, series in [("gyro_x", gx), ("gyro_y", gy), ("gyro_z", gz)]:
        freqs, spectrum = _axis_fft(series, dt_s)
        peaks = _top_peaks(freqs, spectrum)
        peaks_per_axis.append((label, peaks))
        spectra.append((label, freqs, spectrum))
        print()
        print(f"{label} top peaks (Hz, magnitude):")
        for f, a in peaks:
            print(f"  {f:7.1f} Hz   mag {a:12.1f}")

    candidates: List[float] = []
    for _, peaks in peaks_per_axis:
        for f, _amp in peaks[:3]:
            candidates.append(f)
    notch_centres = _cluster_peaks(candidates)

    print()
    print("Suggested notch filter centres (Hz):")
    if not notch_centres:
        print("  (no dominant peaks found)")
    else:
        for f in notch_centres[:5]:
            print(f"  {f:.1f}")

    # ---------------- plot + report ----------------
    png_path = source_path.with_suffix(".png")
    report_path = source_path.with_name(source_path.stem + "_report.txt")

    fig, axes = plt.subplots(2, 1, figsize=(10, 7))
    t_s = (ts - ts[0]) / 1e6
    axes[0].plot(t_s, gx, label="gyro_x", linewidth=0.8)
    axes[0].plot(t_s, gy, label="gyro_y", linewidth=0.8)
    axes[0].plot(t_s, gz, label="gyro_z", linewidth=0.8)
    axes[0].set_xlabel("time (s)")
    axes[0].set_ylabel("gyro (deg/s)")
    axes[0].set_title(
        f"Time series — motor {motor_under_test} @ DShot {cmd_under_test}"
    )
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="upper right")

    for label, freqs, spectrum in spectra:
        axes[1].plot(freqs, spectrum, label=label, alpha=0.75, linewidth=0.8)
    axes[1].set_xlabel("frequency (Hz)")
    axes[1].set_ylabel("magnitude")
    axes[1].set_title("Vibration spectrum (gyro)")
    axes[1].set_xlim(0, sample_rate / 2)
    axes[1].set_yscale("log")
    axes[1].grid(True, alpha=0.3, which="both")
    axes[1].legend(loc="upper right")

    plt.tight_layout()
    plt.savefig(png_path)
    print(f"\nPlot saved to {png_path}")

    with open(report_path, "w") as f:
        f.write("Motor FFT Report\n")
        f.write("================\n")
        f.write(f"Source           : {source_path}\n")
        f.write(f"Samples          : {len(samples)}\n")
        f.write(f"Sample rate (est): {sample_rate:.1f} Hz\n")
        f.write(f"Motor tested     : {motor_under_test} @ DShot {cmd_under_test}\n\n")
        for label, peaks in peaks_per_axis:
            f.write(f"{label} top peaks:\n")
            for fr, amp in peaks:
                f.write(f"  {fr:7.1f} Hz   mag {amp:12.1f}\n")
            f.write("\n")
        f.write("Suggested notch filter centres (Hz):\n")
        if not notch_centres:
            f.write("  (none)\n")
        else:
            for fr in notch_centres[:5]:
                f.write(f"  {fr:.1f}\n")
    print(f"Report saved to {report_path}")

    if show_plot:
        try:
            plt.show()
        except Exception:
            # Headless / no GUI backend — saved files are still on disk.
            pass
    else:
        plt.close(fig)
    return peaks_per_axis


def analyze_ramp_samples(samples: List[Sample], source_path: Path, *, show_plot: bool = True) -> None:
    """Spectrogram view for a throttle ramp capture.

    The motor command varies during the capture so a single FFT smears motor
    fundamentals across the band. A short-time FFT (spectrogram) instead shows
    frequency content vs time, which makes:
      - motor-RPM peaks   appear as diagonal stripes (scale with throttle)
      - frame resonances  appear as horizontal stripes (fixed with throttle)
    These are different things and need different notch strategies, so the
    visual separation matters.
    """
    if len(samples) < 1024:
        print(f"Only {len(samples)} samples — too few for a useful spectrogram.")
        return

    ts = np.array([s.ts_us for s in samples], dtype=np.float64)
    gx = np.array([s.gx for s in samples])
    gy = np.array([s.gy for s in samples])
    gz = np.array([s.gz for s in samples])
    motor_cmd = np.array([s.motor_cmd for s in samples], dtype=np.float64)
    motor_id = samples[len(samples) // 2].motor_id

    dt_us = np.diff(ts)
    valid = (dt_us > 0) & (dt_us < 100000)
    median_dt_us = float(np.median(dt_us[valid]))
    sample_rate = 1.0e6 / median_dt_us

    # nperseg=256 @ 1 kHz -> ~4 Hz freq resolution, 256 ms window
    nperseg = 256
    noverlap = nperseg * 3 // 4

    fig, axes = plt.subplots(4, 1, figsize=(11, 9), sharex=True,
                             gridspec_kw={"height_ratios": [3, 3, 3, 1]})
    t0 = ts[0]
    for ax, label, data in zip(axes[:3], ("gyro_x", "gyro_y", "gyro_z"), (gx, gy, gz)):
        data = data - np.mean(data)
        f_bins, t_bins, Sxx = spectrogram(data, fs=sample_rate,
                                          nperseg=nperseg, noverlap=noverlap,
                                          window="hann", scaling="spectrum")
        Sxx_db = 10.0 * np.log10(Sxx + 1e-12)
        im = ax.pcolormesh(t_bins, f_bins, Sxx_db, shading="auto", cmap="viridis")
        ax.set_ylabel(f"{label}\nfreq (Hz)")
        ax.set_ylim(0, sample_rate / 2)
        fig.colorbar(im, ax=ax, pad=0.01, fraction=0.04, label="dB")

    t_motor = (ts - t0) / 1e6
    axes[3].plot(t_motor, motor_cmd, color="tab:orange", linewidth=1.2)
    axes[3].set_ylabel("DShot\ncmd")
    axes[3].set_xlabel("time (s)")
    axes[3].grid(True, alpha=0.3)
    axes[3].set_ylim(0, 2100)

    target = "all 4" if motor_id == 0xFF or motor_id == 255 else f"motor {motor_id}"
    fig.suptitle(f"Ramp spectrogram — {target}   ({len(samples)} samples, "
                 f"{sample_rate:.0f} Hz)", fontsize=11)
    plt.tight_layout()
    png_path = source_path.with_suffix(".png")
    plt.savefig(png_path, dpi=120)
    print(f"Spectrogram saved to {png_path}")

    # Find motor-tracking peaks: per-time-slice argmax frequency vs motor_cmd
    # gives a quick "is the fundamental tracking RPM" check, per axis.
    report_path = source_path.with_name(source_path.stem + "_report.txt")
    with open(report_path, "w") as f:
        f.write("Ramp spectrogram report\n=======================\n")
        f.write(f"Source: {source_path}\n")
        f.write(f"Samples: {len(samples)}\n")
        f.write(f"Sample rate: {sample_rate:.1f} Hz\n")
        f.write(f"Target: {target}\n")
        f.write(f"Throttle: {int(motor_cmd.min())} -> {int(motor_cmd.max())} DShot\n\n")
        f.write("How to read the PNG:\n")
        f.write("  * Diagonal stripes that climb with the DShot trace = motor-RPM\n")
        f.write("    related (notch widths must scale with RPM, e.g. dynamic notches).\n")
        f.write("  * Horizontal stripes (don't move with throttle) = frame/structural\n")
        f.write("    resonances. Use fixed notch centres.\n")
    print(f"Report saved to {report_path}")

    if show_plot:
        try:
            plt.show()
        except Exception:
            pass
    else:
        plt.close(fig)


def load_csv_samples(path: Path) -> List[Sample]:
    samples: List[Sample] = []
    with open(path, "r", newline="") as f:
        reader = csv.reader(f)
        for i, row in enumerate(reader):
            if i == 0:
                # Skip header line
                if row and row[0].lower().startswith("timestamp"):
                    continue
            if len(row) != 9:
                continue
            try:
                samples.append(
                    Sample(
                        ts_us=int(row[0]),
                        gx=float(row[1]),
                        gy=float(row[2]),
                        gz=float(row[3]),
                        ax=float(row[4]),
                        ay=float(row[5]),
                        az=float(row[6]),
                        motor_id=int(row[7]),
                        motor_cmd=int(row[8]),
                    )
                )
            except ValueError:
                continue
    return samples


# =============================================================================
# Interactive session
# =============================================================================
def run_interactive(port: str, baud: int) -> None:
    ser = serial.Serial(port, baud, timeout=0.1)
    print(f"Opened {port} @ {baud}")
    logger = Logger(ser)
    time.sleep(0.5)
    logger.send("HELP")

    print()
    print("Local commands:")
    print("  test <id 1..4> <dshot 0|48..2047>   send TEST_MOTOR")
    print("  all  <dshot 0|48..2047>             send TEST_ALL (4 motors together)")
    print("  sweep [dshot=1100] [secs=5]         automated: M1,M2,M3,M4 then ALL")
    print("  ramp  [start=48] [end=2000] [s=10] [id=0]  throttle ramp + spectrogram")
    print("  stop                                 send STOP")
    print("  log start [filename]                 begin CSV logging")
    print("  log stop                             end logging + run FFT (plot pops)")
    print("  view <csv>                           re-analyze a saved CSV (plot pops)")
    print("  raw <text>                           send raw line to firmware")
    print("  quit | exit                          close session (sends STOP)")
    print()

    try:
        while True:
            try:
                line = input("> ").strip()
            except EOFError:
                break
            except KeyboardInterrupt:
                print()
                break
            if not line:
                continue
            parts = line.split()
            verb = parts[0].lower()
            if verb in ("quit", "exit", "q"):
                break
            if verb == "test" and len(parts) == 3:
                try:
                    mid = int(parts[1])
                    dshot = int(parts[2])
                    logger.send(f"TEST_MOTOR {mid} {dshot}")
                except ValueError:
                    print("usage: test <id> <dshot>")
                continue
            if verb == "all" and len(parts) == 2:
                try:
                    dshot = int(parts[1])
                    logger.send(f"TEST_ALL {dshot}")
                except ValueError:
                    print("usage: all <dshot>")
                continue
            if verb == "sweep":
                try:
                    dshot = int(parts[1]) if len(parts) > 1 else 1100
                    seconds = float(parts[2]) if len(parts) > 2 else 5.0
                except ValueError:
                    print("usage: sweep [dshot=1100] [secs=5]")
                    continue
                logger.run_sweep(dshot, seconds)
                continue
            if verb == "ramp":
                try:
                    dstart = int(parts[1]) if len(parts) > 1 else 48
                    dend = int(parts[2]) if len(parts) > 2 else 2000
                    secs = float(parts[3]) if len(parts) > 3 else 10.0
                    mid = int(parts[4]) if len(parts) > 4 else 0
                except ValueError:
                    print("usage: ramp [start=48] [end=2000] [secs=10] [motor_id=0 (all)]")
                    continue
                logger.run_ramp(dstart, dend, secs, mid)
                continue
            if verb == "view" and len(parts) == 2:
                logger.replay_csv(Path(parts[1]))
                continue
            if verb == "stop":
                logger.send("STOP")
                continue
            if verb == "log" and len(parts) >= 2:
                sub = parts[1].lower()
                if sub == "start":
                    logger.start_log(parts[2] if len(parts) > 2 else None)
                elif sub == "stop":
                    logger.stop_log()
                else:
                    print("usage: log start [filename] | log stop")
                continue
            if verb == "raw" and len(parts) >= 2:
                logger.send(" ".join(parts[1:]))
                continue
            if verb == "help":
                logger.send("HELP")
                continue
            print(f"unknown local command: {line}  (use 'raw <line>' to passthrough)")
    finally:
        logger.close()
        print("Closed.")


# =============================================================================
# Entry
# =============================================================================
def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="serial port (e.g. COM7 or /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--analyze",
        type=str,
        help="run FFT on an existing CSV file and exit (no serial connection)",
    )
    args = parser.parse_args()

    if args.analyze:
        path = Path(args.analyze)
        if not path.exists():
            print(f"ERROR: {path} not found", file=sys.stderr)
            return 2
        samples = load_csv_samples(path)
        analyze_samples(samples, path)
        return 0

    if not args.port:
        print("ERROR: --port is required (or use --analyze <file>)", file=sys.stderr)
        return 2

    run_interactive(args.port, args.baud)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
