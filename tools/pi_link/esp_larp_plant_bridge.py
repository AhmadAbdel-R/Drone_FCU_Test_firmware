#!/usr/bin/env python3
"""esp_larp plant-scan UART bridge  (Raspberry Pi side).

ADDITIVE and NON-DESTRUCTIVE: this script IMPORTS the production classifier
(save_classifier_scan_desktop.py) read-only and reuses its exact model,
preprocessing, 6x6 grid, per-patch decision and overall average-probability
logic. It does NOT modify, replace, or re-implement any of that. It only:
  * runs the capture -> classify loop CONTINUOUSLY (the production main() stops
    after MAX_IMAGES; here we keep scanning), and
  * transmits one compact line per scan to the FCU over the Pi's UART.

The production script's own output (annotations, CSV, saved images) is
untouched because we do not call its main(); we reuse its functions. If you
also want the annotated images/CSV, run the production program separately —
this bridge is purely the FCU telemetry feed.

WIRING (both sides 3.3 V, no level shifter):
    Pi GPIO14 TXD (pin 8)  -> FCU GPIO16 (Serial2 RX)
    Pi GPIO15 RXD (pin 10) <- FCU GPIO15 (Serial2 TX)   [optional, unused]
    Pi GND                 -- FCU GND
Enable the Pi UART first: raspi-config -> Interface -> Serial ->
  login shell over serial: NO,  serial hardware: YES   (gives /dev/serial0).

PROTOCOL (one '\n'-terminated ASCII line per scan, matches larp_plant_link.h):
    PLANT,<seq>,<ov>,<G>,<Y>,<R>,<aH>,<aP>,<aR>,<cls36>,<cf72>
      ov  = H healthy / P pest / R rust / U uncertain  (overall)
      G,Y,R = patch color counts
      aH/aP/aR = average class probability, integer percent 0..100
      cls36 = 36 chars '0'healthy/'1'pest/'2'rust, row-major r1c1..r6c6
      cf72  = 36 x 2 hex = RAW top-class confidence percent (00..64), uncapped

Run:  python3 esp_larp_plant_bridge.py            (uses /dev/serial0, 115200)
      python3 esp_larp_plant_bridge.py /dev/ttyAMA0 57600
"""

import sys
import time
import os

import cv2
import numpy as np

# Reuse the PRODUCTION pipeline unchanged. Adjust this path if the production
# script lives elsewhere on the Pi.
sys.path.insert(0, "/home/rabeeh/plant_pipeline")
import save_classifier_scan_desktop as s  # noqa: E402  (production ground truth)

try:
    import serial  # pyserial
except ImportError:
    sys.exit("pyserial not installed:  pip3 install pyserial")

from PIL import Image  # noqa: E402


SERIAL_PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/serial0"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
NUM_THREADS = max(1, int(os.environ.get("PLANT_CV_THREADS", os.cpu_count() or 1)))
INTERVAL_SECONDS = max(0.0, float(os.environ.get("PLANT_SCAN_INTERVAL", "0")))

# Overall single-char code from the production overall decision.
_OVERALL_CODE = {"HEALTHY": "H", "UNHEALTHY - PEST": "P", "UNHEALTHY - RUST": "R"}


def classify_batch(net, pil_crops):
    """Classify all grid crops with the production preprocessing in one forward."""
    rgbs = [
        np.array(crop.convert("RGB"), dtype=np.uint8)
        for crop in pil_crops
    ]
    blob = cv2.dnn.blobFromImages(
        rgbs,
        scalefactor=1.0 / 255.0,
        size=(224, 224),
        swapRB=False,
        crop=False,
    )
    net.setInput(blob)
    started = time.perf_counter()
    probs = np.asarray(net.forward(), dtype=np.float32)
    elapsed = time.perf_counter() - started
    probs = probs.reshape(len(pil_crops), -1)
    if probs.shape != (len(pil_crops), 3):
        raise RuntimeError(f"unexpected model output shape: {probs.shape}")

    results = []
    per_patch_time = elapsed / len(pil_crops)
    for row in probs:
        top_id = int(np.argmax(row))
        results.append({
            "prediction": s.CLASS_NAMES[top_id],
            "confidence": float(row[top_id]),
            "healthy": float(row[0]),
            "pest": float(row[1]),
            "rust": float(row[2]),
            "unhealthy": float(row[1] + row[2]),
            "time": per_patch_time,
        })
    return results, probs.copy(), elapsed


def classify_scalar(net, pil_crops):
    """Production reference path, used once to prove batch equivalence."""
    started = time.perf_counter()
    results = [s.classify(net, crop) for crop in pil_crops]
    elapsed = time.perf_counter() - started
    probs = np.asarray(
        [[r["healthy"], r["pest"], r["rust"]] for r in results],
        dtype=np.float32,
    )
    return results, probs, elapsed


def finish_results(results):
    """Apply the production per-patch margin/color rule without changing it."""
    for res in results:
        status, color, rgb = s.decide_status(res)
        res.update({"status": status, "color": color, "rgb": rgb})
    return results


def build_line(seq, block_results):
    """Serialize one scan using ONLY values the production classifier produced."""
    # Per-patch class + RAW confidence (production 'prediction' / 'confidence').
    cls_map = {"healthy": "0", "pest_attack": "1", "rust": "2"}
    cls = "".join(cls_map.get(r["prediction"], "0") for r in block_results)
    cf = "".join(
        "%02x" % max(0, min(0x64, round(float(r["confidence"]) * 100.0)))
        for r in block_results
    )

    counts = {"GREEN": 0, "YELLOW": 0, "RED": 0}
    for r in block_results:
        counts[r["color"]] += 1

    # Overall decision — the SAME average-probability rule as production main().
    avg = {
        k: sum(r[k] for r in block_results) / len(block_results)
        for k in ("healthy", "pest", "rust")
    }
    if avg["rust"] >= s.RUST_AVG_THRESHOLD:
        overall = "UNHEALTHY - RUST"
    elif avg["pest"] >= s.PEST_AVG_THRESHOLD:
        overall = "UNHEALTHY - PEST"
    else:
        overall = "HEALTHY"
    ov = _OVERALL_CODE.get(overall, "U")

    def pct(x):
        return max(0, min(100, round(x * 100.0)))

    return (
        f"PLANT,{seq & 0xFFFF},{ov},"
        f"{counts['GREEN']},{counts['YELLOW']},{counts['RED']},"
        f"{pct(avg['healthy'])},{pct(avg['pest'])},{pct(avg['rust'])},"
        f"{cls},{cf}\n"
    )


def main():
    print(f"esp_larp plant bridge -> {SERIAL_PORT} @ {BAUD}")
    cv2.setNumThreads(NUM_THREADS)
    print(f"OpenCV threads: {cv2.getNumThreads()}; scan interval: {INTERVAL_SECONDS:g}s")
    ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1)
    net = s.load_model()
    cam = s.start_camera()
    seq = 0
    use_batch = None
    last_scan_end = None
    try:
        while True:  # keep scanning (production main() stops after MAX_IMAGES)
            seq += 1
            try:
                scan_started = time.perf_counter()
                capture_started = scan_started
                s.capture_image(cam)
                with Image.open(s.CAPTURE_PATH) as captured:
                    base = captured.convert("RGB")
                if s.ROTATE_DEGREES:
                    base = base.rotate(s.ROTATE_DEGREES, expand=True)
                capture_elapsed = time.perf_counter() - capture_started

                crops = [base.crop(box) for box in s.grid_boxes(*base.size)]
                if use_batch is None:
                    scalar_results, scalar_probs, scalar_elapsed = classify_scalar(net, crops)
                    try:
                        batch_results, batch_probs, batch_elapsed = classify_batch(net, crops)
                        exact = np.array_equal(scalar_probs, batch_probs)
                        same_classes = all(
                            a["prediction"] == b["prediction"]
                            for a, b in zip(scalar_results, batch_results)
                        )
                        max_abs = float(np.max(np.abs(scalar_probs - batch_probs)))
                        use_batch = exact and same_classes
                        speedup = (
                            scalar_elapsed / batch_elapsed
                            if batch_elapsed else float("inf")
                        )
                        print(
                            "Batch verification: "
                            f"exact_probs={exact}, same_classes={same_classes}, "
                            f"max_abs_diff={max_abs:.9g}; "
                            f"scalar={scalar_elapsed * 1000:.1f}ms, "
                            f"batch={batch_elapsed * 1000:.1f}ms, speedup={speedup:.2f}x"
                        )
                    except Exception as batch_error:
                        use_batch = False
                        print(f"Batch verification failed: {batch_error}")
                    if use_batch:
                        block_results = finish_results(batch_results)
                        inference_elapsed = batch_elapsed
                    else:
                        print("Batch is not bit-identical; retaining production scalar inference.")
                        block_results = finish_results(scalar_results)
                        inference_elapsed = scalar_elapsed
                elif use_batch:
                    block_results, _, inference_elapsed = classify_batch(net, crops)
                    finish_results(block_results)
                else:
                    block_results, _, inference_elapsed = classify_scalar(net, crops)
                    finish_results(block_results)

                line = build_line(seq, block_results)
                ser.write(line.encode("ascii"))
                ser.flush()
                scan_end = time.perf_counter()
                scan_elapsed = scan_end - scan_started
                active_hz = 1.0 / scan_elapsed if scan_elapsed else float("inf")
                loop_hz = (
                    1.0 / (scan_end - last_scan_end)
                    if last_scan_end is not None and scan_end != last_scan_end
                    else active_hz
                )
                last_scan_end = scan_end
                mode = "batch" if use_batch else "scalar"
                print(
                    f"[{seq}] {mode} capture={capture_elapsed * 1000:.1f}ms "
                    f"inference={inference_elapsed * 1000:.1f}ms "
                    f"scan={scan_elapsed * 1000:.1f}ms "
                    f"active={active_hz:.2f}Hz loop={loop_hz:.2f}Hz "
                    f"sent: {line.strip()[:60]}..."
                )
            except Exception as e:  # a bad frame must not kill the loop
                print(f"[{seq}] scan error (skipped): {e}")

            if INTERVAL_SECONDS:
                time.sleep(INTERVAL_SECONDS)
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        s.stop_camera(cam)
        ser.close()
        print("Camera + serial released.")


if __name__ == "__main__":
    main()
