#!/usr/bin/env python3
"""
Offline tester for detect_object_real_task3.py — no ROS required.

Runs the same HSV red-pillow detection logic as the live node on one or more
image files (e.g. iPhone photos).  For each image it prints what would be
published to each /perception/* topic and saves an annotated copy.

Usage:
    python detect_object_real_task3_test.py image1.jpg image2.jpg ...
    python detect_object_real_task3_test.py *.jpg

Outputs:
    <original_name>_detected.jpg   — annotated image with bounding box
    (printed to terminal)          — simulated topic values

Dependencies: opencv-python, numpy  (no ROS, no ultralytics)
"""

import sys
import os
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# HSV parameters — mirror the ROS node defaults
# ---------------------------------------------------------------------------
HUE_LOW1  = 0      # lower red band: hue 0–10
HUE_HIGH1 = 10
HUE_LOW2  = 170    # upper red band: hue 170–180
HUE_HIGH2 = 180
SAT_MIN   = 80     # minimum saturation  (0–255)
VAL_MIN   = 60     # minimum brightness  (0–255)
MIN_AREA_PX = 500  # minimum contour area in pixels


# ---------------------------------------------------------------------------
# Detection — verbatim copy of ObjectDetectorNode.detect_target()
# ---------------------------------------------------------------------------

def detect_target(
    bgr: np.ndarray,
    hue_low1: int = HUE_LOW1,
    hue_high1: int = HUE_HIGH1,
    hue_low2: int = HUE_LOW2,
    hue_high2: int = HUE_HIGH2,
    sat_min: int = SAT_MIN,
    val_min: int = VAL_MIN,
    min_area_px: int = MIN_AREA_PX,
) -> Optional[Tuple[int, int, int, int, float, str]]:
    """Return (x, y, w, h, conf, label) for the largest red contour, or None.

    Confidence = fraction of bounding-box pixels that are red (0.0–1.0).
    Identical to ObjectDetectorNode.detect_target() in detect_object_real_task3.py.
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    # Lower red band (hue wraps near 0)
    mask1 = cv2.inRange(
        hsv,
        np.array([hue_low1, sat_min, val_min]),
        np.array([hue_high1, 255, 255]),
    )
    # Upper red band (hue wraps near 180)
    mask2 = cv2.inRange(
        hsv,
        np.array([hue_low2, sat_min, val_min]),
        np.array([hue_high2, 255, 255]),
    )
    mask = cv2.bitwise_or(mask1, mask2)

    # Morphological cleanup
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    best_cnt = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(best_cnt)
    if area < min_area_px:
        return None

    rx, ry, rw, rh = cv2.boundingRect(best_cnt)

    bbox_mask = mask[ry:ry + rh, rx:rx + rw]
    red_pixels = int(np.count_nonzero(bbox_mask))
    total_pixels = rw * rh
    conf = float(red_pixels) / float(max(total_pixels, 1))

    return (rx, ry, rw, rh, conf, "pillow")


# ---------------------------------------------------------------------------
# Annotated image — mirrors ObjectDetectorNode.publish_debug()
# ---------------------------------------------------------------------------

def draw_detection(bgr: np.ndarray, detection: Optional[Tuple], mask_bgr: np.ndarray) -> np.ndarray:
    """Draw bounding box + labels on a copy of the image.  Also shows the HSV mask."""
    vis = bgr.copy()

    if detection is not None:
        x, y, w, h, conf, label = detection
        cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 220, 0), 3)
        text = f"{label}  conf={conf:.2f}"
        cv2.putText(vis, text, (x, max(30, y - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 220, 0), 2)
        # centroid dot
        cx_dot = x + w // 2
        cy_dot = y + h // 2
        cv2.circle(vis, (cx_dot, cy_dot), 6, (0, 0, 255), -1)
        cv2.putText(
            vis,
            f"centroid ({cx_dot}, {cy_dot})",
            (cx_dot + 8, cy_dot),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2,
        )
    else:
        cv2.putText(vis, "NOT DETECTED", (20, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.4, (0, 0, 255), 3)

    # Append the HSV mask side-by-side (resized to same height)
    h_img = vis.shape[0]
    mask_resized = cv2.resize(mask_bgr, (int(mask_bgr.shape[1] * h_img / mask_bgr.shape[0]), h_img))
    combined = np.hstack([vis, mask_resized])
    return combined


def build_mask_bgr(bgr: np.ndarray) -> np.ndarray:
    """Return the red HSV mask as a 3-channel BGR image for display."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m1 = cv2.inRange(hsv, np.array([HUE_LOW1, SAT_MIN, VAL_MIN]), np.array([HUE_HIGH1, 255, 255]))
    m2 = cv2.inRange(hsv, np.array([HUE_LOW2, SAT_MIN, VAL_MIN]), np.array([HUE_HIGH2, 255, 255]))
    mask = cv2.bitwise_or(m1, m2)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)


# ---------------------------------------------------------------------------
# Simulate topic output prints
# ---------------------------------------------------------------------------

def print_topic_outputs(path: str, detection: Optional[Tuple]):
    print(f"\n{'='*60}")
    print(f"  IMAGE: {Path(path).name}")
    print(f"{'='*60}")
    if detection is None:
        print("  /perception/object_found        → False")
        print("  /perception/object_label        → ''")
        print("  /perception/object_confidence   → 0.00")
        print("  /perception/object_bbox         → [0, 0, 0, 0]")
        print("  /perception/object_pose         → (no depth — skipped in test)")
        print("  RESULT: PILLOW NOT DETECTED")
    else:
        x, y, w, h, conf, label = detection
        print(f"  /perception/object_found        → True")
        print(f"  /perception/object_label        → '{label}'")
        print(f"  /perception/object_confidence   → {conf:.4f}  ({conf*100:.1f}% red fill in bbox)")
        print(f"  /perception/object_bbox         → [x={x}, y={y}, w={w}, h={h}]")
        print(f"  /perception/object_pose         → (no depth — skipped in test)")
        cx, cy = x + w // 2, y + h // 2
        print(f"  Centroid pixel                  → ({cx}, {cy})")
        print(f"  Bounding box area               → {w*h} px  (contour fill: {conf*100:.1f}%)")
        print(f"  RESULT: PILLOW DETECTED  ✓")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

IMAGE_FOLDER = Path(__file__).parent / "pillow_test_images"
IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def main():
    # If no args given, scan the pillow_test_images folder automatically
    if len(sys.argv) < 2:
        if not IMAGE_FOLDER.exists():
            print(f"Image folder not found: {IMAGE_FOLDER}")
            sys.exit(1)
        image_paths = sorted(
            str(p) for p in IMAGE_FOLDER.iterdir()
            if p.suffix.lower() in IMAGE_EXTS and "_detected" not in p.stem
        )
        if not image_paths:
            print(f"No images found in {IMAGE_FOLDER}")
            print(f"Drop .jpg / .png files there and run again.")
            sys.exit(0)
        print(f"Found {len(image_paths)} image(s) in {IMAGE_FOLDER}\n")
    else:
        image_paths = sys.argv[1:]

    results = []

    for path in image_paths:
        if not os.path.exists(path):
            print(f"[SKIP] File not found: {path}")
            continue

        bgr = cv2.imread(path)
        if bgr is None:
            print(f"[SKIP] Could not read image: {path}")
            continue

        detection = detect_target(bgr)
        mask_bgr = build_mask_bgr(bgr)
        annotated = draw_detection(bgr, detection, mask_bgr)

        # Save annotated image next to original
        p = Path(path)
        out_path = str(p.parent / (p.stem + "_detected" + p.suffix))
        cv2.imwrite(out_path, annotated)

        print_topic_outputs(path, detection)
        print(f"  Annotated image saved → {out_path}")

        results.append((path, detection))

    # Summary
    total = len(results)
    found = sum(1 for _, d in results if d is not None)
    print(f"\n{'='*60}")
    print(f"  SUMMARY: {found}/{total} images — pillow detected")
    if total > 0:
        confs = [d[4] for _, d in results if d is not None]
        if confs:
            print(f"  Confidence range: {min(confs):.2f} – {max(confs):.2f}  (mean {sum(confs)/len(confs):.2f})")
        print()
        print("  Tuning tips if detection is wrong:")
        print("  - Pillow detected but confidence low  → lower SAT_MIN / VAL_MIN")
        print("  - Red-ish background falsely detected → raise MIN_AREA_PX or SAT_MIN")
        print("  - Pillow not detected at all          → widen HUE_LOW1/HUE_HIGH1 or HUE_LOW2/HUE_HIGH2")
        print("  Current HSV params:")
        print(f"    hue_low1={HUE_LOW1}  hue_high1={HUE_HIGH1}")
        print(f"    hue_low2={HUE_LOW2}  hue_high2={HUE_HIGH2}")
        print(f"    sat_min={SAT_MIN}    val_min={VAL_MIN}")
        print(f"    min_area_px={MIN_AREA_PX}")
    print(f"{'='*60}\n")


if __name__ == "__main__":
    main()
