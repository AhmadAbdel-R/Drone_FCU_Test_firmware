#!/usr/bin/env python3
"""Motor Sweep GUI — desktop app for the fcu_motor_fft_test firmware.

Launches a PyQt6 main window with tabs for:
  - Connection / Live Test (live gyro + DShot plots, sweep buttons, STOP)
  - Spectrum (per-axis FFT)
  - Spectrogram (frequency-vs-time waterfall + DShot overlay)
  - Comparison (load 5 sweep CSVs, side-by-side analysis)
  - Report (auto-generated text summary, save button)

Serial I/O runs in a QThread so the GUI stays responsive. Sample plotting
uses pyqtgraph (Qt-native, much faster than matplotlib for live data).

SAFETY: the GUI's STOP button and any error path send STOP to the firmware.
The firmware separately enforces "only spin what's been explicitly armed".
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
import pyqtgraph as pg
import pyqtgraph.exporters  # registers pg.exporters.ImageExporter
import serial
import serial.tools.list_ports
from PyQt6.QtCore import (
    Qt,
    QObject,
    QRectF,
    QThread,
    QTimer,
    pyqtSignal,
    pyqtSlot,
)
from PyQt6.QtGui import QColor, QFont, QPalette
from PyQt6.QtWidgets import (
    QApplication,
    QComboBox,
    QFileDialog,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QSizePolicy,
    QSpinBox,
    QStatusBar,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)
from scipy.fft import rfft, rfftfreq
from scipy.signal import find_peaks, spectrogram


# =============================================================================
# Constants
# =============================================================================
CSV_HEADER = [
    "timestamp_us",
    "test_target",
    "motor_id",
    "dshot_cmd",
    "phase",
    "gx_raw",
    "gy_raw",
    "gz_raw",
    "ax_raw",
    "ay_raw",
    "az_raw",
    "sample_count",
]

SWEEP_TARGETS = ["M1", "M2", "M3", "M4", "ALL"]
TARGET_LABELS = {
    "M1": "Motor 1",
    "M2": "Motor 2",
    "M3": "Motor 3",
    "M4": "Motor 4",
    "ALL": "All 4 Motors",
}

LIVE_BUFFER_SAMPLES = 120_000  # enough for long full-range stepped sweeps
LIVE_PLOT_MAX_POINTS = 12_000  # downsample display only; saved CSV keeps all rows
SERIAL_BATCH_MIN_SAMPLES = 128
SERIAL_BATCH_MAX_LATENCY_S = 0.050
SWEEP_DEFAULT_MAX_DSHOT = 250
SWEEP_HARD_MAX_DSHOT = 350
SWEEP_DEFAULT_STEP_DSHOT = 10
SWEEP_DEFAULT_HOLD_MS = 500

pg.setConfigOptions(antialias=True, useOpenGL=False, background="w", foreground="k")


# =============================================================================
# Data record
# =============================================================================
@dataclass
class Sample:
    ts_us: int
    target: str
    motor_id: int
    dshot: int
    phase: str
    gx: float
    gy: float
    gz: float
    ax: float
    ay: float
    az: float
    sample_count: int


@dataclass
class Session:
    """One sweep's worth of samples."""

    target: str = ""
    started_at: datetime = field(default_factory=datetime.now)
    samples: List[Sample] = field(default_factory=list)
    source_file: Optional[Path] = None

    def clear(self) -> None:
        self.samples = []
        self.target = ""
        self.started_at = datetime.now()
        self.source_file = None

    def as_numpy(self):
        if not self.samples:
            return None
        ts = np.array([s.ts_us for s in self.samples], dtype=np.float64)
        return {
            "ts_s": (ts - ts[0]) * 1e-6,
            "ts_us": ts,
            "gx": np.array([s.gx for s in self.samples]),
            "gy": np.array([s.gy for s in self.samples]),
            "gz": np.array([s.gz for s in self.samples]),
            "ax": np.array([s.ax for s in self.samples]),
            "ay": np.array([s.ay for s in self.samples]),
            "az": np.array([s.az for s in self.samples]),
            "dshot": np.array([s.dshot for s in self.samples], dtype=np.float64),
            "sample_count": np.array([s.sample_count for s in self.samples], dtype=np.int64),
            "phase": [s.phase for s in self.samples],
            "target": self.samples[len(self.samples) // 2].target if self.samples else "?",
        }

    def estimated_rate_hz(self) -> float:
        if len(self.samples) < 8:
            return 0.0
        ts = np.array([s.ts_us for s in self.samples], dtype=np.float64)
        dt = np.diff(ts)
        dt = dt[(dt > 0) & (dt < 100000)]
        if len(dt) == 0:
            return 0.0
        return 1.0e6 / float(np.median(dt))

    def save_csv(self, path: Path) -> None:
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(CSV_HEADER)
            for s in self.samples:
                w.writerow(
                    [
                        s.ts_us,
                        s.target,
                        s.motor_id,
                        s.dshot,
                        s.phase,
                        f"{s.gx:.3f}",
                        f"{s.gy:.3f}",
                        f"{s.gz:.3f}",
                        f"{s.ax:.3f}",
                        f"{s.ay:.3f}",
                        f"{s.az:.3f}",
                        s.sample_count,
                    ]
                )
        self.source_file = path

    @classmethod
    def from_csv(cls, path: Path) -> "Session":
        sess = cls()
        sess.source_file = path
        with open(path, "r", newline="") as f:
            reader = csv.reader(f)
            for i, row in enumerate(reader):
                if i == 0 and row and row[0].lower().startswith("timestamp"):
                    continue
                if len(row) != 12:
                    continue
                try:
                    sess.samples.append(
                        Sample(
                            ts_us=int(row[0]),
                            target=row[1],
                            motor_id=int(row[2]),
                            dshot=int(row[3]),
                            phase=row[4],
                            gx=float(row[5]),
                            gy=float(row[6]),
                            gz=float(row[7]),
                            ax=float(row[8]),
                            ay=float(row[9]),
                            az=float(row[10]),
                            sample_count=int(row[11]),
                        )
                    )
                except ValueError:
                    continue
        if sess.samples:
            sess.target = sess.samples[len(sess.samples) // 2].target
        return sess


# =============================================================================
# Serial reader thread
# =============================================================================
class SerialWorker(QObject):
    """Runs in its own QThread. Owns the serial port, parses incoming lines,
    emits sample / status / connection signals to the main thread.
    """

    sample = pyqtSignal(object)            # Sample, legacy single-row signal
    samples = pyqtSignal(object)           # list[Sample], batched for high-rate CSV
    status_line = pyqtSignal(str)          # raw status string from firmware
    connection_changed = pyqtSignal(bool, str)  # (connected, message)
    sweep_complete = pyqtSignal()          # firmware signalled "SWEEP complete"

    def __init__(self) -> None:
        super().__init__()
        self._serial: Optional[serial.Serial] = None
        self._port = ""
        self._baud = 115200
        self._running = False
        self._pending_samples: List[Sample] = []
        self._last_sample_flush_s = time.monotonic()

    # ---- Lifecycle (main thread invokes via QMetaObject) ----------------
    @pyqtSlot(str, int)
    def open(self, port: str, baud: int) -> None:
        if self._serial is not None:
            self.close()
        try:
            self._serial = serial.Serial(port, baud, timeout=0.05)
            self._port = port
            self._baud = baud
            self._running = True
            self.connection_changed.emit(True, f"Connected to {port} @ {baud}")
            # Kick off the read loop. We use a timer so the event loop stays
            # serviceable for incoming send_command() calls.
            QTimer.singleShot(0, self._poll)
        except Exception as e:
            self.connection_changed.emit(False, f"Open failed: {e}")
            self._serial = None

    @pyqtSlot()
    def close(self) -> None:
        self._running = False
        if self._serial is not None:
            try:
                self._serial.write(b"STOP\n")
                self._serial.flush()
            except Exception:
                pass
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None
            self.connection_changed.emit(False, "Disconnected")

    @pyqtSlot(str)
    def send_command(self, cmd: str) -> None:
        if self._serial is None:
            return
        if not cmd.endswith("\n"):
            cmd += "\n"
        try:
            self._serial.write(cmd.encode())
            self._serial.flush()
        except Exception as e:
            self.status_line.emit(f"[ERROR] write failed: {e}")
            self._panic()

    # ---- Internal -------------------------------------------------------
    def _panic(self) -> None:
        """Force-disconnect on serial error."""
        self._running = False
        try:
            if self._serial is not None:
                self._serial.close()
        except Exception:
            pass
        self._serial = None
        self.connection_changed.emit(False, "Serial error — disconnected")

    def _poll(self) -> None:
        if not self._running or self._serial is None:
            return
        try:
            while self._serial.in_waiting > 0:
                raw = self._serial.readline()
                if not raw:
                    break
                self._handle_line(raw)
            self._flush_samples(force=False)
        except Exception as e:
            self.status_line.emit(f"[ERROR] read failed: {e}")
            self._panic()
            return
        # Re-arm the poll. 5 ms keeps latency low for 1 kHz CSV streams.
        QTimer.singleShot(5, self._poll)

    def _handle_line(self, raw: bytes) -> None:
        try:
            line = raw.decode(errors="replace").rstrip("\r\n")
        except Exception:
            return
        if not line:
            return
        if line.startswith("[CSV] "):
            payload = line[len("[CSV] "):]
            parts = payload.split(",")
            if len(parts) != 12:
                return  # silently drop malformed rows
            try:
                sample = Sample(
                    ts_us=int(parts[0]),
                    target=parts[1],
                    motor_id=int(parts[2]),
                    dshot=int(parts[3]),
                    phase=parts[4],
                    gx=float(parts[5]),
                    gy=float(parts[6]),
                    gz=float(parts[7]),
                    ax=float(parts[8]),
                    ay=float(parts[9]),
                    az=float(parts[10]),
                    sample_count=int(parts[11]),
                )
            except (ValueError, IndexError):
                return
            self._pending_samples.append(sample)
            if len(self._pending_samples) >= SERIAL_BATCH_MIN_SAMPLES:
                self._flush_samples(force=True)
        else:
            if "SWEEP complete" in line:
                self._flush_samples(force=True)
            self.status_line.emit(line)
            if "SWEEP complete" in line:
                self.sweep_complete.emit()

    def _flush_samples(self, force: bool = False) -> None:
        if not self._pending_samples:
            return
        now_s = time.monotonic()
        if (
            not force
            and len(self._pending_samples) < SERIAL_BATCH_MIN_SAMPLES
            and (now_s - self._last_sample_flush_s) < SERIAL_BATCH_MAX_LATENCY_S
        ):
            return
        batch = self._pending_samples
        self._pending_samples = []
        self._last_sample_flush_s = now_s
        self.samples.emit(batch)


# =============================================================================
# Analysis helpers
# =============================================================================
def axis_fft(values: np.ndarray, fs: float, hp_hz: float = 10.0):
    """Return (freqs, magnitude) for a single-axis time series.

    DC + sub-10 Hz drift is zeroed out so the plot's auto-scale isn't
    dominated by a huge bin at f≈0.
    """
    if len(values) < 32:
        return np.array([]), np.array([])
    v = values - np.mean(values)
    win = np.hanning(len(v))
    spectrum = np.abs(rfft(v * win))
    freqs = rfftfreq(len(v), d=1.0 / fs)
    spectrum[freqs < hp_hz] = 0.0
    return freqs, spectrum


def top_peaks(freqs: np.ndarray, mag: np.ndarray, k: int = 5):
    if len(mag) < 8:
        return []
    idx, _ = find_peaks(mag, height=float(np.max(mag) * 0.1), distance=4)
    order = np.argsort(mag[idx])[::-1][:k]
    return [(float(freqs[idx[i]]), float(mag[idx[i]])) for i in order]


def cluster_peaks(peaks_hz: List[float], min_gap_hz: float = 6.0) -> List[float]:
    if not peaks_hz:
        return []
    pks = sorted(peaks_hz)
    out = [pks[0]]
    for f in pks[1:]:
        if abs(f - out[-1]) > min_gap_hz:
            out.append(f)
    return out


# =============================================================================
# Live test tab
# =============================================================================
class LiveTestTab(QWidget):
    request_sweep = pyqtSignal(str, int, int, int)
    request_stop = pyqtSignal()
    request_save_csv = pyqtSignal()
    request_save_png = pyqtSignal()

    def __init__(
        self,
        sweep_hard_max: int = SWEEP_HARD_MAX_DSHOT,
        default_max_dshot: Optional[int] = None,
    ) -> None:
        super().__init__()
        self.sweep_hard_max = max(48, int(sweep_hard_max))
        self.default_max_dshot = min(
            self.sweep_hard_max,
            max(48, int(default_max_dshot if default_max_dshot is not None else SWEEP_DEFAULT_MAX_DSHOT)),
        )
        main = QVBoxLayout(self)

        # --- Status banner ---------------------------------------------------
        banner = QFrame()
        banner.setFrameShape(QFrame.Shape.StyledPanel)
        bl = QGridLayout(banner)
        self.lbl_target = self._stat_label("—")
        self.lbl_phase = self._stat_label("IDLE")
        self.lbl_elapsed = self._stat_label("0.0 s")
        self.lbl_dshot = self._stat_label("0")
        self.lbl_count = self._stat_label("0")
        self.lbl_rate = self._stat_label("— Hz")
        for col, (caption, lbl) in enumerate(
            [
                ("Target", self.lbl_target),
                ("Phase", self.lbl_phase),
                ("Elapsed", self.lbl_elapsed),
                ("DShot", self.lbl_dshot),
                ("Samples", self.lbl_count),
                ("Sample rate", self.lbl_rate),
            ]
        ):
            cap = QLabel(caption)
            cap.setStyleSheet("color: #666; font-size: 11px;")
            bl.addWidget(cap, 0, col, alignment=Qt.AlignmentFlag.AlignCenter)
            bl.addWidget(lbl, 1, col, alignment=Qt.AlignmentFlag.AlignCenter)
        main.addWidget(banner)

        # --- Sweep parameters -----------------------------------------------
        param_box = QGroupBox("Stepped sweep limits")
        param_row = QHBoxLayout(param_box)
        self.spin_max = QSpinBox()
        self.spin_max.setRange(48, self.sweep_hard_max)
        self.spin_max.setValue(self.default_max_dshot)
        self.spin_max.setSuffix(" DShot max")
        self.spin_step = QSpinBox()
        self.spin_step.setRange(1, 100)
        self.spin_step.setValue(SWEEP_DEFAULT_STEP_DSHOT)
        self.spin_step.setSuffix(" step")
        self.spin_hold = QSpinBox()
        self.spin_hold.setRange(100, 5000)
        self.spin_hold.setSingleStep(50)
        self.spin_hold.setValue(SWEEP_DEFAULT_HOLD_MS)
        self.spin_hold.setSuffix(" ms hold")
        param_row.addWidget(QLabel("Max:"))
        param_row.addWidget(self.spin_max)
        param_row.addWidget(QLabel("Step:"))
        param_row.addWidget(self.spin_step)
        param_row.addWidget(QLabel("Hold:"))
        param_row.addWidget(self.spin_hold)
        param_row.addStretch()
        main.addWidget(param_box)

        # --- Buttons row -----------------------------------------------------
        btn_row = QHBoxLayout()
        self.btn_m1 = self._make_button("Run Motor 1 Sweep", "#1F8AC0")
        self.btn_m2 = self._make_button("Run Motor 2 Sweep", "#1F8AC0")
        self.btn_m3 = self._make_button("Run Motor 3 Sweep", "#1F8AC0")
        self.btn_m4 = self._make_button("Run Motor 4 Sweep", "#1F8AC0")
        self.btn_all = self._make_button("Run All Motors Sweep", "#E89A00")
        for b, t in [
            (self.btn_m1, "M1"),
            (self.btn_m2, "M2"),
            (self.btn_m3, "M3"),
            (self.btn_m4, "M4"),
            (self.btn_all, "ALL"),
        ]:
            b.clicked.connect(lambda _checked, tgt=t: self._emit_sweep(tgt))
            btn_row.addWidget(b)

        # STOP button — big, red, always visible
        self.btn_stop = QPushButton("STOP")
        self.btn_stop.setMinimumHeight(64)
        self.btn_stop.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        f = QFont()
        f.setBold(True)
        f.setPointSize(20)
        self.btn_stop.setFont(f)
        self.btn_stop.setStyleSheet(
            "QPushButton { background-color: #C62828; color: white; border-radius: 8px; }"
            "QPushButton:hover { background-color: #D63838; }"
        )
        self.btn_stop.clicked.connect(self.request_stop.emit)
        btn_row.addWidget(self.btn_stop, stretch=2)
        main.addLayout(btn_row)

        # --- Live plots ------------------------------------------------------
        self.gw = pg.GraphicsLayoutWidget()
        self.plot_gx = self.gw.addPlot(row=0, col=0, title="gyro_x (raw, dps)")
        self.plot_gy = self.gw.addPlot(row=1, col=0, title="gyro_y (raw, dps)")
        self.plot_gz = self.gw.addPlot(row=2, col=0, title="gyro_z (raw, dps)")
        self.plot_dshot = self.gw.addPlot(row=3, col=0, title="DShot command")
        for p in (self.plot_gx, self.plot_gy, self.plot_gz, self.plot_dshot):
            p.showGrid(x=True, y=True, alpha=0.3)
            p.setLabel("bottom", "time (s)")
        self.plot_dshot.setYRange(-20, self.sweep_hard_max + 50)

        self.curve_gx = self.plot_gx.plot(pen=pg.mkPen("#C62828", width=1))
        self.curve_gy = self.plot_gy.plot(pen=pg.mkPen("#1F8AC0", width=1))
        self.curve_gz = self.plot_gz.plot(pen=pg.mkPen("#2A8F3A", width=1))
        self.curve_dshot = self.plot_dshot.plot(pen=pg.mkPen("#E89A00", width=2))
        main.addWidget(self.gw, stretch=1)

        # --- Save row --------------------------------------------------------
        save_row = QHBoxLayout()
        self.btn_save_csv = QPushButton("Save CSV")
        self.btn_save_csv.clicked.connect(self.request_save_csv.emit)
        self.btn_save_png = QPushButton("Save Live Plot PNG")
        self.btn_save_png.clicked.connect(self.request_save_png.emit)
        save_row.addWidget(self.btn_save_csv)
        save_row.addWidget(self.btn_save_png)
        save_row.addStretch()
        main.addLayout(save_row)

        # Ring buffers for live plotting. Avoid shifting large arrays while a
        # high-rate CSV stream is active; display is downsampled at draw time.
        self._buf_t = np.zeros(LIVE_BUFFER_SAMPLES)
        self._buf_gx = np.zeros(LIVE_BUFFER_SAMPLES)
        self._buf_gy = np.zeros(LIVE_BUFFER_SAMPLES)
        self._buf_gz = np.zeros(LIVE_BUFFER_SAMPLES)
        self._buf_d = np.zeros(LIVE_BUFFER_SAMPLES)
        self._buf_write = 0
        self._buf_count = 0
        self._t0_us = None

        # Plot refresh is throttled — we don't redraw on every sample. 30 Hz
        # is smooth visually and leaves CPU for serial parsing.
        self._refresh_pending = False
        self._timer = QTimer(self)
        self._timer.setInterval(33)
        self._timer.timeout.connect(self._flush_plots)
        self._timer.start()

    def _make_button(self, text: str, color: str) -> QPushButton:
        b = QPushButton(text)
        b.setMinimumHeight(48)
        f = QFont()
        f.setPointSize(11)
        f.setBold(True)
        b.setFont(f)
        b.setStyleSheet(
            f"QPushButton {{ background-color: {color}; color: white; border-radius: 6px; }}"
            f"QPushButton:disabled {{ background-color: #aaa; }}"
        )
        return b

    def _emit_sweep(self, target: str) -> None:
        self.request_sweep.emit(
            target,
            int(self.spin_max.value()),
            int(self.spin_step.value()),
            int(self.spin_hold.value()),
        )

    def _stat_label(self, text: str) -> QLabel:
        l = QLabel(text)
        f = QFont()
        f.setPointSize(14)
        f.setBold(True)
        l.setFont(f)
        return l

    @pyqtSlot()
    def reset_plots(self) -> None:
        self._buf_write = 0
        self._buf_count = 0
        self._t0_us = None
        for c in (self.curve_gx, self.curve_gy, self.curve_gz, self.curve_dshot):
            c.setData([], [])
        self.lbl_target.setText("—")
        self.lbl_phase.setText("IDLE")
        self.lbl_elapsed.setText("0.0 s")
        self.lbl_dshot.setText("0")
        self.lbl_count.setText("0")
        self.lbl_rate.setText("— Hz")

    @pyqtSlot(object)
    def add_sample(self, s: Sample) -> None:
        if self._t0_us is None:
            self._t0_us = s.ts_us
        t = (s.ts_us - self._t0_us) * 1e-6

        i = self._buf_used
        if i >= LIVE_BUFFER_SAMPLES:
            # Roll: shift left by 1/8 of buffer, keep newest portion.
            shift = LIVE_BUFFER_SAMPLES // 8
            for arr in (self._buf_t, self._buf_gx, self._buf_gy, self._buf_gz, self._buf_d):
                arr[:-shift] = arr[shift:]
            self._buf_used -= shift
            i = self._buf_used
        self._buf_t[i] = t
        self._buf_gx[i] = s.gx
        self._buf_gy[i] = s.gy
        self._buf_gz[i] = s.gz
        self._buf_d[i] = s.dshot
        self._buf_used += 1

        # Status banner (cheap text update, do it every sample)
        self.lbl_target.setText(TARGET_LABELS.get(s.target, s.target or "—"))
        self.lbl_phase.setText(s.phase or "IDLE")
        self.lbl_elapsed.setText(f"{t:5.1f} s")
        self.lbl_dshot.setText(str(s.dshot))
        self.lbl_count.setText(f"{s.sample_count}")
        # Estimate rate from the last 256 samples in the live buffer
        if self._buf_used >= 64:
            window = self._buf_t[max(0, self._buf_used - 256):self._buf_used]
            dt = np.diff(window)
            dt = dt[dt > 0]
            if len(dt) > 4:
                self.lbl_rate.setText(f"{1.0 / float(np.median(dt)):.0f} Hz")

        self._refresh_pending = True

    def _flush_plots(self) -> None:
        if not self._refresh_pending:
            return
        n = self._buf_used
        if n == 0:
            return
        t = self._buf_t[:n]
        self.curve_gx.setData(t, self._buf_gx[:n])
        self.curve_gy.setData(t, self._buf_gy[:n])
        self.curve_gz.setData(t, self._buf_gz[:n])
        self.curve_dshot.setData(t, self._buf_d[:n])
        self._refresh_pending = False

    # High-rate replacement methods. These intentionally appear after the
    # original simple methods so Python binds these versions on the class.
    @pyqtSlot(object)
    def add_sample(self, s: Sample) -> None:
        self.add_samples([s])

    @pyqtSlot(object)
    def add_samples(self, samples: List[Sample]) -> None:
        if not samples:
            return
        if self._t0_us is None:
            self._t0_us = samples[0].ts_us

        for s in samples:
            t = (s.ts_us - self._t0_us) * 1e-6
            i = self._buf_write
            self._buf_t[i] = t
            self._buf_gx[i] = s.gx
            self._buf_gy[i] = s.gy
            self._buf_gz[i] = s.gz
            self._buf_d[i] = s.dshot
            self._buf_write = (self._buf_write + 1) % LIVE_BUFFER_SAMPLES
            if self._buf_count < LIVE_BUFFER_SAMPLES:
                self._buf_count += 1

        last = samples[-1]
        t_last = (last.ts_us - self._t0_us) * 1e-6
        self.lbl_target.setText(TARGET_LABELS.get(last.target, last.target or "-"))
        self.lbl_phase.setText(last.phase or "IDLE")
        self.lbl_elapsed.setText(f"{t_last:5.1f} s")
        self.lbl_dshot.setText(str(last.dshot))
        self.lbl_count.setText(f"{last.sample_count}")

        if self._buf_count >= 64:
            window = self._ordered_array(self._buf_t)[-256:]
            dt = np.diff(window)
            dt = dt[dt > 0]
            if len(dt) > 4:
                self.lbl_rate.setText(f"{1.0 / float(np.median(dt)):.0f} Hz")

        self._refresh_pending = True

    def _ordered_array(self, arr: np.ndarray) -> np.ndarray:
        if self._buf_count == 0:
            return arr[:0]
        if self._buf_count < LIVE_BUFFER_SAMPLES:
            return arr[:self._buf_count]
        return np.concatenate((arr[self._buf_write:], arr[:self._buf_write]))

    def _flush_plots(self) -> None:
        if not self._refresh_pending:
            return
        n = self._buf_count
        if n == 0:
            return
        t = self._ordered_array(self._buf_t)
        gx = self._ordered_array(self._buf_gx)
        gy = self._ordered_array(self._buf_gy)
        gz = self._ordered_array(self._buf_gz)
        d = self._ordered_array(self._buf_d)
        stride = max(1, int(math.ceil(n / LIVE_PLOT_MAX_POINTS)))
        if stride > 1:
            t = t[::stride]
            gx = gx[::stride]
            gy = gy[::stride]
            gz = gz[::stride]
            d = d[::stride]
        t, gx, gy, gz, d = self._break_plot_gaps(t, gx, gy, gz, d)
        self.curve_gx.setData(t, gx)
        self.curve_gy.setData(t, gy)
        self.curve_gz.setData(t, gz)
        self.curve_dshot.setData(t, d)
        self._refresh_pending = False

    def _break_plot_gaps(self, t: np.ndarray, *series: np.ndarray):
        if len(t) < 2:
            return (t, *series)
        gap_idx = np.flatnonzero(np.diff(t) > 0.25) + 1
        if len(gap_idx) == 0:
            return (t, *series)
        t_out = np.insert(t, gap_idx, np.nan)
        return (t_out, *(np.insert(values, gap_idx, np.nan) for values in series))


# =============================================================================
# Spectrum tab
# =============================================================================
class SpectrumTab(QWidget):
    def __init__(self) -> None:
        super().__init__()
        v = QVBoxLayout(self)
        self.info = QLabel("Run a sweep or load a CSV to see the spectrum.")
        self.info.setStyleSheet("color: #444; padding: 8px;")
        v.addWidget(self.info)

        self.gw = pg.GraphicsLayoutWidget()
        self.plot_x = self.gw.addPlot(row=0, col=0, title="gyro_x FFT magnitude")
        self.plot_y = self.gw.addPlot(row=1, col=0, title="gyro_y FFT magnitude")
        self.plot_z = self.gw.addPlot(row=2, col=0, title="gyro_z FFT magnitude")
        for p in (self.plot_x, self.plot_y, self.plot_z):
            p.showGrid(x=True, y=True, alpha=0.3)
            p.setLabel("bottom", "frequency (Hz)")
            p.setLogMode(False, True)  # log-Y; linear-X
        self.curve_x = self.plot_x.plot(pen=pg.mkPen("#C62828", width=1))
        self.curve_y = self.plot_y.plot(pen=pg.mkPen("#1F8AC0", width=1))
        self.curve_z = self.plot_z.plot(pen=pg.mkPen("#2A8F3A", width=1))
        v.addWidget(self.gw)

        save_row = QHBoxLayout()
        self.btn_save_png = QPushButton("Save Spectrum PNG")
        self.btn_save_png.clicked.connect(self._save_png)
        save_row.addWidget(self.btn_save_png)
        save_row.addStretch()
        v.addLayout(save_row)

        self._current_session: Optional[Session] = None
        self._rendered_session_id: Optional[int] = None

    def set_session(self, session: Session, defer: bool = False) -> None:
        self._current_session = session
        self._rendered_session_id = None
        if defer:
            self.info.setText(
                f"Target {session.target}  |  {len(session.samples)} samples  |  "
                "plot will render when this tab is opened"
            )
            return
        self._render_session(session)

    def ensure_rendered(self) -> None:
        if self._current_session is None:
            return
        if self._rendered_session_id == id(self._current_session):
            return
        self._render_session(self._current_session)

    def _render_session(self, session: Session) -> None:
        data = session.as_numpy()
        if data is None:
            self.info.setText("No samples yet.")
            return
        fs = session.estimated_rate_hz()
        if fs < 1:
            self.info.setText("Could not estimate sample rate.")
            return
        nyq = fs / 2.0
        xmax = min(500.0, nyq)
        peaks_text = []
        for label, vals, curve, plot in (
            ("gyro_x", data["gx"], self.curve_x, self.plot_x),
            ("gyro_y", data["gy"], self.curve_y, self.plot_y),
            ("gyro_z", data["gz"], self.curve_z, self.plot_z),
        ):
            f, m = axis_fft(vals, fs)
            curve.setData(f, m)
            plot.setXRange(0, xmax)
            pk = top_peaks(f, m, k=3)
            peaks_text.append(
                f"{label}: " + (", ".join(f"{fr:.0f} Hz" for fr, _ in pk) or "no peaks")
            )
        self.info.setText(
            f"Target {session.target}  |  {len(session.samples)} samples  |  "
            f"fs ≈ {fs:.0f} Hz  |  top peaks  →  " + "   ·   ".join(peaks_text)
        )
        self._rendered_session_id = id(session)

    def _save_png(self) -> None:
        self.ensure_rendered()
        path, _ = QFileDialog.getSaveFileName(
            self, "Save spectrum PNG", self._suggested_name(), "PNG (*.png)"
        )
        if not path:
            return
        exporter = pg.exporters.ImageExporter(self.gw.scene())
        exporter.export(path)
        QMessageBox.information(self, "Saved", f"Spectrum PNG saved to {path}")

    def _suggested_name(self) -> str:
        if self._current_session and self._current_session.source_file:
            return str(self._current_session.source_file.with_suffix(".spectrum.png"))
        return f"spectrum_{datetime.now():%Y%m%d_%H%M%S}.png"


# =============================================================================
# Spectrogram tab
# =============================================================================
class SpectrogramTab(QWidget):
    def __init__(self) -> None:
        super().__init__()
        v = QVBoxLayout(self)
        self.info = QLabel("Run a sweep or load a CSV to see the spectrogram.")
        self.info.setStyleSheet("color: #444; padding: 8px;")
        v.addWidget(self.info)

        self.gw = pg.GraphicsLayoutWidget()
        self.plots = []
        self.images = []
        for i, label in enumerate(("gyro_x", "gyro_y", "gyro_z")):
            p = self.gw.addPlot(row=i, col=0, title=f"{label} spectrogram")
            p.setLabel("left", "freq (Hz)")
            p.setLabel("bottom", "time (s)")
            img = pg.ImageItem()
            p.addItem(img)
            cmap = pg.colormap.get("viridis")
            img.setLookupTable(cmap.getLookupTable())
            self.plots.append(p)
            self.images.append(img)
        self.plot_dshot = self.gw.addPlot(row=3, col=0, title="DShot command")
        self.plot_dshot.setLabel("bottom", "time (s)")
        self.plot_dshot.showGrid(x=True, y=True, alpha=0.3)
        self.curve_dshot = self.plot_dshot.plot(pen=pg.mkPen("#E89A00", width=2))
        v.addWidget(self.gw)

        save_row = QHBoxLayout()
        self.btn_save_png = QPushButton("Save Spectrogram PNG")
        self.btn_save_png.clicked.connect(self._save_png)
        save_row.addWidget(self.btn_save_png)
        save_row.addStretch()
        v.addLayout(save_row)

        self._current_session: Optional[Session] = None
        self._rendered_session_id: Optional[int] = None

    def set_session(self, session: Session, defer: bool = False) -> None:
        self._current_session = session
        self._rendered_session_id = None
        if defer:
            self.info.setText(
                f"Target {session.target}  |  {len(session.samples)} samples  |  "
                "plot will render when this tab is opened"
            )
            return
        self._render_session(session)

    def ensure_rendered(self) -> None:
        if self._current_session is None:
            return
        if self._rendered_session_id == id(self._current_session):
            return
        self._render_session(self._current_session)

    def _render_session(self, session: Session) -> None:
        data = session.as_numpy()
        if data is None:
            self.info.setText("No samples yet.")
            return
        fs = session.estimated_rate_hz()
        if fs < 1:
            self.info.setText("Could not estimate sample rate.")
            return
        nperseg = min(256, max(64, len(data["gx"]) // 32))
        nover = nperseg * 3 // 4
        xmax_freq = min(500.0, fs / 2.0)

        for img, plot, vals in zip(
            self.images, self.plots, (data["gx"], data["gy"], data["gz"])
        ):
            if len(vals) < nperseg:
                continue
            v_dc_removed = vals - np.mean(vals)
            f, t, Sxx = spectrogram(
                v_dc_removed, fs=fs, nperseg=nperseg, noverlap=nover, window="hann"
            )
            Sxx_db = 10 * np.log10(Sxx + 1e-12)
            img.setImage(Sxx_db.T, autoLevels=True)
            # Map image pixel coords to (time, frequency)
            img.setRect(QRectF(float(t[0]), float(f[0]),
                               float(t[-1] - t[0]), float(f[-1] - f[0])))
            plot.setYRange(0, xmax_freq)

        self.curve_dshot.setData(data["ts_s"], data["dshot"])
        self._rendered_session_id = id(session)
        self.info.setText(
            f"Target {session.target}  |  fs ≈ {fs:.0f} Hz  |  "
            f"FFT window {nperseg} samples ({nperseg/fs*1000:.0f} ms)"
        )

    def _save_png(self) -> None:
        self.ensure_rendered()
        path, _ = QFileDialog.getSaveFileName(
            self, "Save spectrogram PNG", self._suggested_name(), "PNG (*.png)"
        )
        if not path:
            return
        exporter = pg.exporters.ImageExporter(self.gw.scene())
        exporter.export(path)
        QMessageBox.information(self, "Saved", f"Spectrogram PNG saved to {path}")

    def _suggested_name(self) -> str:
        if self._current_session and self._current_session.source_file:
            return str(self._current_session.source_file.with_suffix(".spectrogram.png"))
        return f"spectrogram_{datetime.now():%Y%m%d_%H%M%S}.png"


# =============================================================================
# Comparison tab
# =============================================================================
class ComparisonTab(QWidget):
    def __init__(self) -> None:
        super().__init__()
        v = QVBoxLayout(self)
        v.addWidget(QLabel(
            "Load up to 5 sweep CSVs (one per motor + ALL) to compare their spectra "
            "and identify shared peaks."
        ))

        # File pickers per target
        form = QFormLayout()
        self.file_labels: dict[str, QLabel] = {}
        self.pick_buttons: dict[str, QPushButton] = {}
        self.loaded: dict[str, Session] = {}
        for tgt in SWEEP_TARGETS:
            lbl = QLabel("(no file loaded)")
            lbl.setStyleSheet("color: #888;")
            btn = QPushButton(f"Pick CSV for {TARGET_LABELS[tgt]}…")
            btn.clicked.connect(lambda _c, t=tgt: self._pick(t))
            row = QHBoxLayout()
            row.addWidget(btn, stretch=1)
            row.addWidget(lbl, stretch=3)
            wrap = QWidget()
            wrap.setLayout(row)
            form.addRow(wrap)
            self.file_labels[tgt] = lbl
            self.pick_buttons[tgt] = btn
        v.addLayout(form)

        self.gw = pg.GraphicsLayoutWidget()
        self.plot_x = self.gw.addPlot(row=0, col=0, title="gyro_x — overlaid spectra")
        self.plot_y = self.gw.addPlot(row=1, col=0, title="gyro_y — overlaid spectra")
        self.plot_z = self.gw.addPlot(row=2, col=0, title="gyro_z — overlaid spectra")
        for p in (self.plot_x, self.plot_y, self.plot_z):
            p.showGrid(x=True, y=True, alpha=0.3)
            p.setLogMode(False, True)
            p.addLegend()
            p.setLabel("bottom", "frequency (Hz)")
        v.addWidget(self.gw, stretch=1)

        self.summary = QPlainTextEdit()
        self.summary.setReadOnly(True)
        self.summary.setMaximumHeight(160)
        self.summary.setStyleSheet("font-family: Consolas, monospace;")
        v.addWidget(self.summary)

    def _pick(self, target: str) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, f"CSV for {TARGET_LABELS[target]}", "", "CSV files (*.csv)"
        )
        if not path:
            return
        try:
            sess = Session.from_csv(Path(path))
        except Exception as e:
            QMessageBox.warning(self, "Load failed", str(e))
            return
        self.loaded[target] = sess
        self.file_labels[target].setText(f"{Path(path).name}  ({len(sess.samples)} samples)")
        self.file_labels[target].setStyleSheet("color: #1F4674;")
        self._redraw()

    def _redraw(self) -> None:
        colors = {"M1": "#C62828", "M2": "#1F8AC0", "M3": "#2A8F3A", "M4": "#7A2AAF", "ALL": "#E89A00"}
        for plot in (self.plot_x, self.plot_y, self.plot_z):
            plot.clear()
            plot.addLegend()

        all_peaks_per_target: dict[str, List[float]] = {}
        per_axis_top: dict[str, dict[str, Tuple[float, float]]] = {"gx": {}, "gy": {}, "gz": {}}
        for tgt in SWEEP_TARGETS:
            if tgt not in self.loaded:
                continue
            sess = self.loaded[tgt]
            data = sess.as_numpy()
            if data is None:
                continue
            fs = sess.estimated_rate_hz()
            if fs < 1:
                continue
            color = colors[tgt]
            for plot, vals, axkey in (
                (self.plot_x, data["gx"], "gx"),
                (self.plot_y, data["gy"], "gy"),
                (self.plot_z, data["gz"], "gz"),
            ):
                f, m = axis_fft(vals, fs)
                plot.plot(f, m, pen=pg.mkPen(color, width=1), name=tgt)
                plot.setXRange(0, min(500.0, fs / 2.0))
                pk = top_peaks(f, m, k=3)
                if pk:
                    per_axis_top[axkey][tgt] = pk[0]
                    all_peaks_per_target.setdefault(tgt, []).extend(fr for fr, _ in pk)

        # Summary
        lines = []
        if not self.loaded:
            lines.append("Load at least one CSV to see the analysis.")
        else:
            lines.append("Strongest peak per axis per motor (Hz, magnitude):")
            for axkey in ("gx", "gy", "gz"):
                row = [f"  {axkey}:"]
                for tgt in SWEEP_TARGETS:
                    if tgt in per_axis_top[axkey]:
                        f0, mag = per_axis_top[axkey][tgt]
                        row.append(f"  {tgt}={f0:5.1f}/{mag:9.1f}")
                lines.append("".join(row))

            # Noisiest motor (sum of top-axis magnitudes)
            noise_scores = {}
            for tgt in SWEEP_TARGETS:
                if tgt not in self.loaded:
                    continue
                total = 0.0
                for axkey in ("gx", "gy", "gz"):
                    if tgt in per_axis_top[axkey]:
                        total += per_axis_top[axkey][tgt][1]
                noise_scores[tgt] = total
            if noise_scores:
                worst = max(noise_scores, key=noise_scores.get)
                lines.append(f"\nNoisiest target (sum of top peaks across axes): {TARGET_LABELS[worst]}")

            # Shared peaks across all loaded targets
            shared_pool = []
            for pks in all_peaks_per_target.values():
                shared_pool.extend(pks)
            clustered = cluster_peaks(shared_pool, min_gap_hz=8.0)
            lines.append("\nClustered peak frequencies seen across loaded sweeps (candidate notches):")
            for f0 in clustered[:8]:
                lines.append(f"  {f0:6.1f} Hz")

            if "ALL" in self.loaded:
                all_data = self.loaded["ALL"].as_numpy()
                if all_data is not None and self.loaded["ALL"].estimated_rate_hz() > 0:
                    fs_all = self.loaded["ALL"].estimated_rate_hz()
                    combined = []
                    for axkey in ("gx", "gy", "gz"):
                        f, m = axis_fft(all_data[axkey], fs_all)
                        for fr, _ in top_peaks(f, m, k=3):
                            combined.append(fr)
                    notch = cluster_peaks(combined, min_gap_hz=10.0)
                    if notch:
                        lo, hi = min(notch), max(notch)
                        lines.append(
                            f"\nSuggested notch range from ALL sweep: {lo:.0f}–{hi:.0f} Hz "
                            f"(centres: {', '.join(f'{f:.0f}' for f in notch[:5])} Hz)"
                        )

        self.summary.setPlainText("\n".join(lines))


# =============================================================================
# Report tab
# =============================================================================
class ReportTab(QWidget):
    def __init__(self) -> None:
        super().__init__()
        v = QVBoxLayout(self)
        v.addWidget(QLabel("Auto-generated report. Click Save to write to disk."))
        self.text = QPlainTextEdit()
        self.text.setReadOnly(True)
        self.text.setStyleSheet("font-family: Consolas, monospace; font-size: 11pt;")
        v.addWidget(self.text)
        row = QHBoxLayout()
        self.btn_save = QPushButton("Save Report (.txt)")
        self.btn_save.clicked.connect(self._save)
        row.addWidget(self.btn_save)
        self.btn_save_md = QPushButton("Save Report (.md)")
        self.btn_save_md.clicked.connect(lambda: self._save(md=True))
        row.addWidget(self.btn_save_md)
        row.addStretch()
        v.addLayout(row)
        self._session: Optional[Session] = None

    def set_session(self, session: Session) -> None:
        self._session = session
        data = session.as_numpy()
        if data is None:
            self.text.setPlainText("No samples to report.")
            return
        fs = session.estimated_rate_hz()
        n = len(session.samples)
        target = session.target
        ramp_dshot = data["dshot"]
        ramp_started = session.started_at.strftime("%Y-%m-%d %H:%M:%S")
        sample_count = data["sample_count"]
        expected_samples = int(sample_count[-1] - sample_count[0] + 1) if n else 0
        missing_samples = max(0, expected_samples - n)
        saved_fraction = (n / expected_samples) if expected_samples > 0 else 0.0
        sample_jumps = np.diff(sample_count) if n > 1 else np.array([], dtype=np.int64)
        max_missing_run = int(np.max(sample_jumps - 1)) if len(sample_jumps) else 0
        ts_gaps_s = np.diff(data["ts_us"]) * 1e-6 if n > 1 else np.array([])
        max_gap_s = float(np.max(ts_gaps_s)) if len(ts_gaps_s) else 0.0
        capture_has_dropouts = (
            saved_fraction < 0.98 or missing_samples > 100 or max_gap_s > 0.25
        )

        # Per-axis top peaks
        per_axis = {}
        for axkey in ("gx", "gy", "gz"):
            f, m = axis_fft(data[axkey], fs)
            per_axis[axkey] = top_peaks(f, m, k=5)

        # Identify motor-related vs resonance bands.
        # Motor-related = peak that moves with dshot — approximated here by
        # checking if a peak appears strongly only in some time bins.
        # We do a simple spectrogram-band split: peaks that exist at low
        # DShot (start/end of sweep) are likely frame; peaks that only
        # appear at high DShot are likely motor RPM.
        motor_band: List[float] = []
        frame_band: List[float] = []
        if n >= 256:
            half = n // 2
            for axkey in ("gx", "gy", "gz"):
                f_lo, m_lo = axis_fft(data[axkey][: n // 4], fs)
                f_hi, m_hi = axis_fft(data[axkey][n // 2 : n // 2 + n // 4], fs)
                pk_lo = {round(fr / 5) * 5 for fr, _ in top_peaks(f_lo, m_lo, k=3)}
                pk_hi = {round(fr / 5) * 5 for fr, _ in top_peaks(f_hi, m_hi, k=3)}
                # Peaks only at high RPM = motor-related
                motor_band.extend(sorted(pk_hi - pk_lo))
                # Peaks present in both = frame resonance
                frame_band.extend(sorted(pk_hi & pk_lo))

        clean = (
            "yes"
            if (
                n > fs * 25
                and ramp_dshot.max() > 1500
                and ramp_dshot.min() < 50
                and not capture_has_dropouts
            )
            else "incomplete"
        )

        lines = [
            "MOTOR SWEEP REPORT",
            "=" * 60,
            f"Test target            : {TARGET_LABELS.get(target, target)}",
            f"Captured at            : {ramp_started}",
            f"Sample count           : {n}",
            f"Saved sample fraction  : {saved_fraction * 100:.1f}% ({missing_samples} missing)",
            f"Max capture gap        : {max_gap_s:.3f}s ({max_missing_run} missing samples)",
            f"Effective sample rate  : {fs:.1f} Hz",
            f"Ramp DShot min / max   : {int(ramp_dshot.min())} / {int(ramp_dshot.max())}",
            f"Duration               : {data['ts_s'][-1]:.1f} s",
            f"Sweep looks clean?     : {clean}",
            "",
            "Strongest gyro peaks (Hz, magnitude):",
        ]
        for axkey in ("gx", "gy", "gz"):
            lines.append(f"  {axkey}:")
            for fr, mag in per_axis[axkey]:
                lines.append(f"    {fr:7.1f} Hz   mag {mag:10.1f}")
        lines.append("")
        lines.append("Likely motor-RPM bands (peaks appearing at high throttle only):")
        if motor_band:
            for f0 in cluster_peaks([float(x) for x in motor_band], min_gap_hz=8.0)[:5]:
                lines.append(f"  {f0:.0f} Hz")
        else:
            lines.append("  (none cleanly identified — recapture with smooth ramp)")
        lines.append("")
        lines.append("Likely frame-resonance bands (peaks present at low AND high throttle):")
        if frame_band:
            for f0 in cluster_peaks([float(x) for x in frame_band], min_gap_hz=8.0)[:5]:
                lines.append(f"  {f0:.0f} Hz")
        else:
            lines.append("  (none cleanly identified)")
        lines.append("")
        all_pk = [fr for axkey in per_axis for fr, _ in per_axis[axkey][:2]]
        notch = cluster_peaks(all_pk, min_gap_hz=10.0)
        if notch:
            lines.append("Suggested notch centres (Hz):")
            for f0 in notch[:5]:
                lines.append(f"  {f0:.0f}")
        else:
            lines.append("Suggested notch centres: (insufficient data)")
        lines.append("")
        lines.append("Notes:")
        if capture_has_dropouts:
            lines.append("  - Capture dropouts detected; FFT/spectrogram peaks may be distorted.")
        lines.append("  - Motor-band peaks should be covered by wide notches or dynamic notch filters.")
        lines.append("  - Frame-resonance peaks need narrow notches at the exact centre frequency.")
        lines.append("  - Re-run the sweep AFTER applying filters to verify the peaks are attenuated.")
        self.text.setPlainText("\n".join(lines))

    def _save(self, md: bool = False) -> None:
        if self._session is None:
            QMessageBox.information(self, "Nothing to save", "No report yet.")
            return
        ext = "md" if md else "txt"
        suggested = (
            str(self._session.source_file.with_suffix(f"_report.{ext}"))
            if self._session.source_file
            else f"sweep_{self._session.target.lower()}_{datetime.now():%Y%m%d_%H%M%S}_report.{ext}"
        )
        path, _ = QFileDialog.getSaveFileName(
            self, "Save report", suggested, f"{ext.upper()} files (*.{ext})"
        )
        if not path:
            return
        Path(path).write_text(self.text.toPlainText())
        QMessageBox.information(self, "Saved", f"Report saved to {path}")


# =============================================================================
# Main window
# =============================================================================
class MainWindow(QMainWindow):
    cmd_open = pyqtSignal(str, int)
    cmd_close = pyqtSignal()
    cmd_send = pyqtSignal(str)

    def __init__(
        self,
        sweep_hard_max: int = SWEEP_HARD_MAX_DSHOT,
        default_max_dshot: Optional[int] = None,
    ) -> None:
        super().__init__()
        self.setWindowTitle("FCU Motor Sweep — GUI")
        self.resize(1400, 900)

        self._current_session = Session()
        self._connected = False
        self._sweep_hard_max = max(48, int(sweep_hard_max))

        # ---- Top connection panel ------------------------------------------
        conn_box = QGroupBox("Connection")
        cl = QHBoxLayout(conn_box)
        cl.addWidget(QLabel("Port:"))
        self.cb_port = QComboBox()
        self.cb_port.setMinimumWidth(180)
        cl.addWidget(self.cb_port)
        self.btn_refresh = QPushButton("Refresh")
        self.btn_refresh.clicked.connect(self._refresh_ports)
        cl.addWidget(self.btn_refresh)
        cl.addWidget(QLabel("Baud:"))
        self.cb_baud = QComboBox()
        for b in ("921600", "460800", "230400", "115200"):
            self.cb_baud.addItem(b)
        cl.addWidget(self.cb_baud)
        self.btn_connect = QPushButton("Connect")
        self.btn_connect.clicked.connect(self._toggle_connect)
        cl.addWidget(self.btn_connect)
        self.lbl_conn = QLabel("● Disconnected")
        self.lbl_conn.setStyleSheet("color: #C62828; font-weight: bold;")
        cl.addWidget(self.lbl_conn)
        cl.addStretch()

        # ---- Tabs -----------------------------------------------------------
        self.tabs = QTabWidget()
        self.tab_live = LiveTestTab(self._sweep_hard_max, default_max_dshot)
        self.tab_spectrum = SpectrumTab()
        self.tab_spectrogram = SpectrogramTab()
        self.tab_compare = ComparisonTab()
        self.tab_report = ReportTab()
        self.tabs.addTab(self.tab_live, "Live Test")
        self.tabs.addTab(self.tab_spectrum, "Spectrum")
        self.tabs.addTab(self.tab_spectrogram, "Spectrogram")
        self.tabs.addTab(self.tab_compare, "Comparison")
        self.tabs.addTab(self.tab_report, "Report")
        self.tabs.currentChanged.connect(self._on_tab_changed)

        central = QWidget()
        cv = QVBoxLayout(central)
        cv.addWidget(conn_box)
        cv.addWidget(self.tabs, stretch=1)
        self.setCentralWidget(central)

        # Status bar with rolling log
        self.setStatusBar(QStatusBar())
        self.statusBar().showMessage("Ready")

        # ---- Serial worker --------------------------------------------------
        self.worker = SerialWorker()
        self.worker_thread = QThread()
        self.worker.moveToThread(self.worker_thread)
        self.worker_thread.start()

        self.cmd_open.connect(self.worker.open)
        self.cmd_close.connect(self.worker.close)
        self.cmd_send.connect(self.worker.send_command)

        self.worker.sample.connect(self._on_sample)
        self.worker.samples.connect(self._on_samples)
        self.worker.status_line.connect(self._on_status)
        self.worker.connection_changed.connect(self._on_conn_change)
        self.worker.sweep_complete.connect(self._on_sweep_complete)

        self.tab_live.request_sweep.connect(self._on_sweep_request)
        self.tab_live.request_stop.connect(self._on_stop_request)
        self.tab_live.request_save_csv.connect(self._save_csv)
        self.tab_live.request_save_png.connect(self._save_live_png)

        self._refresh_ports()

    # ---- Connection -------------------------------------------------------
    def _refresh_ports(self) -> None:
        self.cb_port.clear()
        for info in serial.tools.list_ports.comports():
            self.cb_port.addItem(f"{info.device} — {info.description or ''}", info.device)
        if self.cb_port.count() == 0:
            self.cb_port.addItem("(no ports found)", "")

    def _toggle_connect(self) -> None:
        if self._connected:
            self.cmd_close.emit()
        else:
            port = self.cb_port.currentData()
            if not port:
                QMessageBox.warning(self, "No port", "No serial port selected.")
                return
            baud = int(self.cb_baud.currentText())
            self.cmd_open.emit(port, baud)

    @pyqtSlot(bool, str)
    def _on_conn_change(self, connected: bool, message: str) -> None:
        self._connected = connected
        if connected:
            self.lbl_conn.setText("● Connected")
            self.lbl_conn.setStyleSheet("color: #1F9E41; font-weight: bold;")
            self.btn_connect.setText("Disconnect")
        else:
            self.lbl_conn.setText("● Disconnected")
            self.lbl_conn.setStyleSheet("color: #C62828; font-weight: bold;")
            self.btn_connect.setText("Connect")
        self.statusBar().showMessage(message, 5000)

    # ---- Sample / status routing -----------------------------------------
    @pyqtSlot(object)
    def _on_sample(self, s: Sample) -> None:
        # Start a new session on the first sample of a sweep
        if not self._current_session.samples or self._current_session.target != s.target:
            self._current_session = Session(target=s.target)
        self._current_session.samples.append(s)
        self.tab_live.add_sample(s)

    @pyqtSlot(object)
    def _on_samples(self, samples: List[Sample]) -> None:
        if not samples:
            return
        for s in samples:
            if not self._current_session.samples or self._current_session.target != s.target:
                self._current_session = Session(target=s.target)
            self._current_session.samples.append(s)
        self.tab_live.add_samples(samples)

    @pyqtSlot(str)
    def _on_status(self, line: str) -> None:
        self.statusBar().showMessage(line, 8000)
        # Don't spam the console; the status bar is the user-facing surface

    @pyqtSlot()
    def _on_sweep_complete(self) -> None:
        # Keep the GUI responsive at the end of a high-rate sweep. The heavy
        # plot renders happen lazily when the user opens each analysis tab.
        self.tab_spectrum.set_session(self._current_session, defer=True)
        self.tab_spectrogram.set_session(self._current_session, defer=True)
        QTimer.singleShot(50, lambda: self.tab_report.set_session(self._current_session))
        self.statusBar().showMessage(
            f"Sweep complete: captured {len(self._current_session.samples)} samples",
            8000,
        )

    @pyqtSlot(int)
    def _on_tab_changed(self, _index: int) -> None:
        current = self.tabs.currentWidget()
        if current is self.tab_spectrum:
            self.statusBar().showMessage("Rendering spectrum...", 2000)
            self.tab_spectrum.ensure_rendered()
        elif current is self.tab_spectrogram:
            self.statusBar().showMessage("Rendering spectrogram...", 2000)
            self.tab_spectrogram.ensure_rendered()

    # ---- Sweep controls --------------------------------------------------
    @pyqtSlot(str, int, int, int)
    def _on_sweep_request(self, target: str, max_dshot: int, step_dshot: int, hold_ms: int) -> None:
        if not self._connected:
            QMessageBox.warning(self, "Not connected", "Connect to the FCU first.")
            return
        duration_s = self._estimate_sweep_duration_s(max_dshot, step_dshot, hold_ms)
        ok = QMessageBox.question(
            self,
            f"Run {TARGET_LABELS[target]} sweep?",
            f"Run stepped sweep up to DShot {max_dshot}?\n\n"
            f"Step: {step_dshot} DShot\n"
            f"Hold: {hold_ms} ms\n"
            f"Duration: about {duration_s:.1f} s\n\n"
            "Props OFF. Use a restrained stand for any prop-on test.",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel,
        )
        if ok != QMessageBox.StandardButton.Yes:
            return
        if max_dshot > SWEEP_HARD_MAX_DSHOT:
            ok = QMessageBox.question(
                self,
                "Full DShot range enabled",
                f"This sweep can reach DShot {max_dshot}.\n\n"
                "That is a full-command motor test. Use only on a restrained "
                "test stand with props removed or otherwise controlled.",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel,
            )
            if ok != QMessageBox.StandardButton.Yes:
                return
        if target == "ALL":
            ok = QMessageBox.question(
                self,
                "Run ALL motors?",
                "Run Motor 1+2+3+4 together?\n\n"
                "Airframe MUST be mechanically restrained.\n"
                "Props OFF unless you are in a vibration test stand.",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel,
            )
            if ok != QMessageBox.StandardButton.Yes:
                return
        # Fresh session for this sweep
        self._current_session = Session(target=target)
        self.tab_live.reset_plots()
        self.cmd_send.emit(f"RUN_SWEEP {target} {max_dshot} {step_dshot} {hold_ms}")
        self.statusBar().showMessage(f"Sweep started: {TARGET_LABELS[target]}", 0)

    @staticmethod
    def _estimate_sweep_duration_s(max_dshot: int, step_dshot: int, hold_ms: int) -> float:
        command_count = 1 + max(0, (max_dshot - 48 + step_dshot - 1) // step_dshot)
        slot_count = 2 + (2 * command_count)
        return (slot_count * hold_ms) / 1000.0

    @pyqtSlot()
    def _on_stop_request(self) -> None:
        self.cmd_send.emit("STOP")
        self.statusBar().showMessage("STOP sent", 3000)

    # ---- Save buttons ----------------------------------------------------
    def _save_csv(self) -> None:
        if not self._current_session.samples:
            QMessageBox.information(self, "Nothing to save", "No samples captured yet.")
            return
        ts = self._current_session.started_at.strftime("%Y%m%d_%H%M%S")
        tgt = self._current_session.target.lower() or "session"
        suggested = f"sweep_{tgt}_{ts}.csv"
        path, _ = QFileDialog.getSaveFileName(self, "Save CSV", suggested, "CSV (*.csv)")
        if not path:
            return
        self._current_session.save_csv(Path(path))
        self.tab_spectrum.set_session(self._current_session)
        self.tab_spectrogram.set_session(self._current_session)
        self.tab_report.set_session(self._current_session)
        QMessageBox.information(self, "Saved", f"CSV saved to {path}")

    def _save_live_png(self) -> None:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        suggested = f"sweep_live_{ts}.png"
        path, _ = QFileDialog.getSaveFileName(self, "Save live PNG", suggested, "PNG (*.png)")
        if not path:
            return
        exporter = pg.exporters.ImageExporter(self.tab_live.gw.scene())
        exporter.export(path)
        QMessageBox.information(self, "Saved", f"Live PNG saved to {path}")

    # ---- Lifecycle -------------------------------------------------------
    def closeEvent(self, event) -> None:  # noqa: N802 (Qt naming)
        if self._connected:
            self.cmd_send.emit("STOP")
            time.sleep(0.1)
            self.cmd_close.emit()
        self.worker_thread.quit()
        self.worker_thread.wait(1500)
        super().closeEvent(event)


# =============================================================================
# Entry point
# =============================================================================
def main() -> int:
    parser = argparse.ArgumentParser(description="FCU motor sweep GUI")
    parser.add_argument(
        "--allow-full-dshot",
        action="store_true",
        help=(
            "allow the sweep max control up to 2047. Requires firmware built "
            "with env:fcu_motor_fft_test_full_dshot."
        ),
    )
    parser.add_argument(
        "--default-max-dshot",
        type=int,
        default=SWEEP_DEFAULT_MAX_DSHOT,
        help="initial Max DShot value shown in the Live Test tab",
    )
    args, qt_args = parser.parse_known_args()
    hard_max = 2047 if args.allow_full_dshot else SWEEP_HARD_MAX_DSHOT
    if args.default_max_dshot < 48 or args.default_max_dshot > hard_max:
        parser.error(f"--default-max-dshot must be between 48 and {hard_max}")

    app = QApplication([sys.argv[0], *qt_args])
    app.setStyle("Fusion")
    win = MainWindow(sweep_hard_max=hard_max, default_max_dshot=args.default_max_dshot)
    win.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
