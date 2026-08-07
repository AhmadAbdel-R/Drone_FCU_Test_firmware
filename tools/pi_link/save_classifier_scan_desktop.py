import subprocess
import time
import csv
import signal
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont


ONNX_PATH = Path("/home/rabeeh/best.onnx")

PIPELINE_DIR = Path("/home/rabeeh/plant_pipeline")
CAPTURE_PATH = PIPELINE_DIR / "camera_capture.jpg"
CAMERA_LOG = PIPELINE_DIR / "camera.log"

SAVE_DIR = Path("/home/rabeeh/Desktop/plant_scan_results")
SAVE_DIR.mkdir(parents=True, exist_ok=True)

CLASS_NAMES = {0: "healthy", 1: "pest_attack", 2: "rust"}

# One run = this many photos, then it exits on its own
MAX_IMAGES = 10
INTERVAL_SECONDS = 2

# Camera
CAM_WIDTH = 1280
CAM_HEIGHT = 960
ROTATE_DEGREES = -90   # -90 = 90 clockwise (sideways cam). 0 = off, 90 = other way

# Grid split
GRID_COLS = 6
GRID_ROWS = 6

PEST_AVG_THRESHOLD = 0.80
RUST_AVG_THRESHOLD = 0.70

# Look & feel (0 = invisible, 255 = solid black)
PANEL_ALPHA = 90
LABEL_ALPHA = 100
CELL_TINT_ALPHA = 25
BORDER_PX = 6

# Traffic-light thresholds (applied per block)
GREEN_HEALTHY_CONF = 0.50
RED_UNHEALTHY_CONF = 0.55

DECISION_MARGIN = 0.10

FONT_DIR = Path("/usr/share/fonts/truetype/dejavu")

BLOCK_NAMES = [
    "top_left", "top_center", "top_right",
    "mid_left", "mid_center", "mid_right",
    "bottom_left", "bottom_center", "bottom_right",
]


def load_font(size, bold=False):
    name = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
    try:
        return ImageFont.truetype(str(FONT_DIR / name), size)
    except OSError:
        return ImageFont.load_default()


def start_camera():
    log = open(CAMERA_LOG, "w")
    proc = subprocess.Popen([
        "rpicam-still", "-o", str(CAPTURE_PATH),
        "--width", str(CAM_WIDTH), "--height", str(CAM_HEIGHT),
        "--timeout", "0", "--signal", "--nopreview",
    ], stdout=log, stderr=log)
    time.sleep(2.5)  # camera boot + exposure settle
    if proc.poll() is not None:
        raise RuntimeError(
            "Camera failed to start - probably in use by another program.\n"
            f"Check {CAMERA_LOG} and run:  pgrep -af rpicam"
        )
    return proc


def capture_image(cam):
    if cam.poll() is not None:
        raise RuntimeError(f"Camera process died. Check {CAMERA_LOG}")
    before = CAPTURE_PATH.stat().st_mtime if CAPTURE_PATH.exists() else 0.0
    cam.send_signal(signal.SIGUSR1)
    deadline = time.time() + 6
    while time.time() < deadline:
        if CAPTURE_PATH.exists() and CAPTURE_PATH.stat().st_mtime != before:
            time.sleep(0.25)  # let the file finish writing
            return
        time.sleep(0.05)
    raise RuntimeError(f"Camera did not produce an image in time. Check {CAMERA_LOG}")


def stop_camera(cam):
    if cam.poll() is None:
        try:
            cam.send_signal(signal.SIGUSR2)
            cam.wait(timeout=2)
        except Exception:
            cam.terminate()
            try:
                cam.wait(timeout=2)
            except Exception:
                cam.kill()


def softmax(x):
    x = x - np.max(x)
    e = np.exp(x)
    return e / np.sum(e)


def load_model():
    print("Loading ONNX patch classifier...")
    net = cv2.dnn.readNetFromONNX(str(ONNX_PATH))
    print("Model loaded.")
    return net


def classify(net, pil_img):
    # PIL already provides RGB, matching the model's training input.
    rgb = np.array(pil_img.convert("RGB"), dtype=np.uint8)
    blob = cv2.dnn.blobFromImage(
        rgb,
        scalefactor=1.0 / 255.0,
        size=(224, 224),
        swapRB=False,
        crop=False,
    )

    net.setInput(blob)
    start = time.time()
    probs = np.asarray(net.forward(), dtype=np.float32).reshape(-1)
    elapsed = time.time() - start

    top_id = int(np.argmax(probs))
    return {
        "prediction": CLASS_NAMES[top_id],
        "confidence": float(probs[top_id]),
        "healthy": float(probs[0]),
        "pest": float(probs[1]),
        "rust": float(probs[2]),
        "unhealthy": float(probs[1] + probs[2]),
        "time": elapsed,
    }


def decide_status(res):
    ranked = sorted((res["healthy"], res["pest"], res["rust"]), reverse=True)
    margin = ranked[0] - ranked[1]
    if margin < DECISION_MARGIN:
        return "UNCERTAIN", "YELLOW", (255, 210, 0)
    if res["prediction"] == "healthy":
        return "HEALTHY", "GREEN", (0, 180, 0)
    return "UNHEALTHY", "RED", (255, 0, 0)


def overall_from_counts(counts):
    if counts["RED"] > 0:
        return "UNHEALTHY", "RED", (255, 0, 0)
    if counts["YELLOW"] > 0:
        return "UNCERTAIN", "YELLOW", (255, 210, 0)
    return "HEALTHY", "GREEN", (0, 180, 0)


def grid_boxes(width, height):
    xs = [round(width * c / GRID_COLS) for c in range(GRID_COLS + 1)]
    ys = [round(height * r / GRID_ROWS) for r in range(GRID_ROWS + 1)]
    boxes = []
    for r in range(GRID_ROWS):
        for c in range(GRID_COLS):
            boxes.append((xs[c], ys[r], xs[c + 1], ys[r + 1]))
    return boxes


def display_confidence(raw):
    # Conservative displayed score: preserves raw confidence in CSV/decisions.
    return min(0.90, 0.85 * float(raw) + 0.05)


def annotate_block(crop, res):
    img = crop.convert("RGBA")
    w, h = img.size
    overlay = Image.new("RGBA", img.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(overlay)

    if CELL_TINT_ALPHA > 0:
        d.rectangle((0, 0, w, h), fill=(*res["rgb"], CELL_TINT_ALPHA))

    f = load_font(12, bold=True)
    text = f"{res['prediction'].replace('_', ' ')} {display_confidence(res['confidence']) * 100:.0f}%"
    tw = d.textlength(text, font=f)
    d.rounded_rectangle((6, 6, 6 + tw + 12, 26), radius=6, fill=(0, 0, 0, LABEL_ALPHA))
    d.text((12, 9), text, font=f, fill=(255, 255, 255, 255))

    img = Image.alpha_composite(img, overlay).convert("RGB")
    b = ImageDraw.Draw(img)
    for i in range(3):
        b.rectangle((i, i, w - 1 - i, h - 1 - i), outline=res["rgb"])
    return img


def annotate_full(base_img, block_results, overall_status, overall_rgb, scan_num, counts, total_ms):
    img = base_img.convert("RGBA")
    w, h = img.size
    overlay = Image.new("RGBA", img.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(overlay)

    f_label = load_font(12, bold=True)
    f_status = load_font(17, bold=True)
    f_info = load_font(12)

    for res in block_results:
        x0, y0, x1, y1 = res["box"]
        if CELL_TINT_ALPHA > 0:
            d.rectangle((x0, y0, x1, y1), fill=(*res["rgb"], CELL_TINT_ALPHA))
        text = f"{res['prediction'].replace('_', ' ')} {display_confidence(res['confidence']) * 100:.0f}%"
        tw = d.textlength(text, font=f_label)
        lx, ly = x0 + 7, y0 + 7
        d.rounded_rectangle((lx, ly, lx + tw + 12, ly + 19), radius=6, fill=(0, 0, 0, LABEL_ALPHA))
        d.text((lx + 6, ly + 2), text, font=f_label, fill=(255, 255, 255, 255))

    strip_h = 34
    sx0, sy0 = 14, h - 14 - strip_h
    sx1 = w - 14
    d.rounded_rectangle((sx0, sy0, sx1, sy0 + strip_h), radius=10, fill=(0, 0, 0, PANEL_ALPHA))

    d.text((sx0 + 12, sy0 + 7), overall_status, font=f_status, fill=(*overall_rgb, 255))
    stw = d.textlength(overall_status, font=f_status)

    mid = f"{counts['GREEN']}G / {counts['YELLOW']}Y / {counts['RED']}R"
    d.text((sx0 + 12 + stw + 14, sy0 + 10), mid, font=f_info, fill=(235, 235, 235, 255))

    right = f"scan {scan_num:02d}  {total_ms:.0f} ms  {time.strftime('%H:%M:%S')}"
    rw = d.textlength(right, font=f_info)
    d.text((sx1 - 12 - rw, sy0 + 10), right, font=f_info, fill=(185, 185, 185, 255))

    img = Image.alpha_composite(img, overlay).convert("RGB")

    b = ImageDraw.Draw(img)
    for res in block_results:
        x0, y0, x1, y1 = res["box"]
        for i in range(2):
            b.rectangle((x0 + i, y0 + i, x1 - 1 - i, y1 - 1 - i), outline=res["rgb"])
    for i in range(BORDER_PX):
        b.rectangle((i, i, w - 1 - i, h - 1 - i), outline=overall_rgb)

    return img


def main():
    run_stamp = time.strftime("%Y%m%d_%H%M%S")
    run_dir = SAVE_DIR / f"run_{run_stamp}"
    run_dir.mkdir(parents=True, exist_ok=True)
    csv_path = run_dir / "scan_results.csv"

    print()
    print("Plant classifier - 6x6 ONNX patch scan")
    print("---------------------------------")
    print(f"Taking {MAX_IMAGES} photos, one every {INTERVAL_SECONDS}s.")
    print(f"This run's folder: {run_dir}")
    print("Press Ctrl+C to stop early.")
    print()

    net = load_model()
    print("Starting camera (stays open for the whole run)...")
    cam = start_camera()
    print("Camera ready.")
    print()

    try:
        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "scan", "folder", "block", "position", "status", "color",
                "prediction", "confidence", "healthy", "pest", "rust",
                "unhealthy", "inference_s",
            ])

            scan_num = 0
            try:
                while scan_num < MAX_IMAGES:
                    scan_num += 1
                    scan_stamp = time.strftime("%Y%m%d_%H%M%S")
                    scan_dir = run_dir / f"scan_{scan_num:02d}_{scan_stamp}"
                    scan_dir.mkdir(parents=True, exist_ok=True)

                    capture_image(cam)
                    base = Image.open(CAPTURE_PATH).convert("RGB")
                    if ROTATE_DEGREES:
                        base = base.rotate(ROTATE_DEGREES, expand=True)
                    base.save(scan_dir / "original.jpg", quality=95)

                    block_results = []
                    total_infer = 0.0
                    for i, box in enumerate(grid_boxes(*base.size)):
                        block_name = f"r{i // GRID_COLS + 1}c{i % GRID_COLS + 1}"
                        crop = base.crop(box)
                        res = classify(net, crop)
                        status, color, rgb = decide_status(res)
                        res.update({
                            "status": status, "color": color, "rgb": rgb,
                            "box": box, "index": i + 1, "name": block_name,
                        })
                        total_infer += res["time"]
                        block_results.append(res)

                        annotate_block(crop, res).save(
                            scan_dir / f"block_{i + 1}_{block_name}_{color}.jpg",
                            quality=92,
                        )

                        writer.writerow([
                            scan_num, scan_dir.name, i + 1, block_name,
                            status, color, res["prediction"],
                            f"{res['confidence']:.4f}", f"{res['healthy']:.4f}",
                            f"{res['pest']:.4f}", f"{res['rust']:.4f}",
                            f"{res['unhealthy']:.4f}", f"{res['time']:.4f}",
                        ])

                    counts = {"GREEN": 0, "YELLOW": 0, "RED": 0}
                    for r in block_results:
                        counts[r["color"]] += 1
                    avg = {
                        k: sum(r[k] for r in block_results) / len(block_results)
                        for k in ("healthy", "pest", "rust", "unhealthy")
                    }

                    if avg["rust"] >= RUST_AVG_THRESHOLD:
                        overall_status, overall_color = "UNHEALTHY - RUST", "RED"
                        overall_rgb = (255, 0, 0)
                    elif avg["pest"] >= PEST_AVG_THRESHOLD:
                        overall_status, overall_color = "UNHEALTHY - PEST", "RED"
                        overall_rgb = (255, 0, 0)
                    else:
                        overall_status, overall_color = "HEALTHY", "GREEN"
                        overall_rgb = (0, 180, 0)

                    annotate_full(
                        base, block_results, overall_status, overall_rgb,
                        scan_num, counts, total_infer * 1000,
                    ).save(scan_dir / f"full_{overall_color}.jpg", quality=95)

                    avg = {k: sum(r[k] for r in block_results) / len(block_results)
                           for k in ("healthy", "pest", "rust", "unhealthy")}
                    writer.writerow([
                        scan_num, scan_dir.name, 0, "overall",
                        overall_status, overall_color, "", "",
                        f"{avg['healthy']:.4f}", f"{avg['pest']:.4f}",
                        f"{avg['rust']:.4f}", f"{avg['unhealthy']:.4f}",
                        f"{total_infer:.4f}",
                    ])
                    f.flush()

                    print(f"[{scan_num}/{MAX_IMAGES}] {overall_status} ({overall_color})  "
                          f"G:{counts['GREEN']} Y:{counts['YELLOW']} R:{counts['RED']}  ->  {scan_dir.name}/")

                    if scan_num < MAX_IMAGES:
                        time.sleep(INTERVAL_SECONDS)

            except KeyboardInterrupt:
                print()
                print("Stopped by user.")
    finally:
        stop_camera(cam)
        print("Camera released.")

    print()
    print("Done. Everything from this run is in:")
    print(run_dir)


if __name__ == "__main__":
    main()
