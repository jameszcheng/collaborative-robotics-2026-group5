#!/usr/bin/env python3
"""
Offline tester for detect_object_real_task3.py (HSV red-pillow detection).

No ROS, no depth camera needed.  Drop in phone photos and see:
  - Whether the pillow is detected
  - The annotated image with bounding box
  - Exactly what would be published to each ROS topic

Uses the IDENTICAL HSV detection logic as detect_object_real_task3.py.
Only the depth / TF / publishing steps are skipped (not available offline).

Install (no ROS needed):
    pip install opencv-python numpy

Usage:
    # Single image:
    python3 detect_object_real_task3_test.py --images /path/to/photo.jpg

    # Folder of images:
    python3 detect_object_real_task3_test.py --images /path/to/folder/

    # Tune HSV thresholds:
    python3 detect_object_real_task3_test.py --images photo.jpg \\
        --hue_low1 0 --hue_high1 12 --hue_low2 168 --hue_high2 180 \\
        --sat_min 70 --val_min 50

    # Save annotated images instead of displaying:
    python3 detect_object_real_task3_test.py --images /path/ --save_dir ./results/

    # Don't open display window (headless):
    python3 detect_object_real_task3_test.py --images /path/ --no_display --save_dir ./results/

    # Show the raw HSV mask as well (useful for tuning):
    python3 detect_object_real_task3_test.py --images photo.jpg --show_mask
"""

import argparse
import sys
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np

IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}

LABEL = "pillow"

# ---------------------------------------------------------------------------
# Detection logic — identical to detect_object_real_task3.py detect_target()
# ---------------------------------------------------------------------------

def detect_target(
    bgr: np.ndarray,
    hue_low1: int,
    hue_high1: int,
    hue_low2: int,
    hue_high2: int,
    sat_min: int,
    val_min: int,
    min_area_px: int,
) -> Tuple[Optional[Tuple[int, int, int, int, float, str]], np.ndarray]:
    """Return (x, y, w, h, conf, label) or None, plus the binary mask."""
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

    # Morphological cleanup — remove small noise blobs
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, mask

    # Largest contour by area
    best_cnt = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(best_cnt)
    if area < min_area_px:
        return None, mask

    rx, ry, rw, rh = cv2.boundingRect(best_cnt)

    # Confidence = red-pixel fill ratio inside the bounding box
    bbox_mask = mask[ry:ry + rh, rx:rx + rw]
    red_pixels = int(np.count_nonzero(bbox_mask))
    total_pixels = rw * rh
    conf = float(red_pixels) / float(max(total_pixels, 1))

    return (rx, ry, rw, rh, conf, LABEL), mask


# ---------------------------------------------------------------------------
# Visualisation
# ---------------------------------------------------------------------------

def annotate(bgr: np.ndarray, detection: Optional[Tuple]) -> np.ndarray:
    vis = bgr.copy()
    if detection is None:
        cv2.putText(vis, "NOT DETECTED", (20, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.4, (0, 0, 220), 3)
        return vis

    x, y, w, h, conf, label = detection
    cx, cy = x + w // 2, y + h // 2

    # Bounding box
    cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 220, 0), 3)
    # Centroid cross
    cv2.drawMarker(vis, (cx, cy), (0, 220, 0), cv2.MARKER_CROSS, 20, 2)
    # Label + confidence
    cv2.putText(vis, f"{label}  conf={conf:.2f}", (x, max(30, y - 10)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 220, 0), 2)
    # Pixel centroid info
    cv2.putText(vis, f"centroid px: ({cx}, {cy})", (x, y + h + 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 200, 255), 2)
    return vis


def print_topic_simulation(detection: Optional[Tuple], image_name: str):
    """Print what detect_object_real_task3.py would publish to each ROS topic."""
    print(f"\n  {'─'*54}")
    print(f"  Image: {image_name}")
    print(f"  {'─'*54}")
    if detection is None:
        print(f"  /perception/object_found       → False")
        print(f"  /perception/object_label       → ''")
        print(f"  /perception/object_confidence  → 0.000")
        print(f"  /perception/object_bbox        → [0, 0, 0, 0]")
        print(f"  /perception/object_pose        → (no publish — not found)")
        print(f"  /perception/object_debug_image → (annotated, no bbox)")
        return

    x, y, w, h, conf, label = detection
    cx, cy = x + w // 2, y + h // 2
    print(f"  /perception/object_found       → True")
    print(f"  /perception/object_label       → '{label}'")
    print(f"  /perception/object_confidence  → {conf:.3f}  (red fill ratio of bbox)")
    print(f"  /perception/object_bbox        → [{x}, {y}, {w}, {h}]")
    print(f"  /perception/object_pose        → would need depth + TF (skipped offline)")
    print(f"  /perception/object_debug_image → (annotated image, see window / save_dir)")
    print(f"\n  Bounding box : top-left=({x},{y})  size={w}×{h}px")
    print(f"  Centroid px  : ({cx}, {cy})")
    print(f"  Contour area : {w*h} px² (bbox), fill={conf*100:.1f}% red")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def find_images(input_path: str):
    p = Path(input_path)
    if p.is_file():
        return [p]
    imgs = sorted(f for f in p.iterdir() if f.suffix.lower() in IMAGE_EXTENSIONS)
    if not imgs:
        print(f"No images found in {input_path}")
        sys.exit(1)
    return imgs


def show_image(title: str, img: np.ndarray, max_w: int = 1280):
    h, w = img.shape[:2]
    if w > max_w:
        img = cv2.resize(img, (max_w, int(h * max_w / w)))
    cv2.imshow(title, img)


def main():
    parser = argparse.ArgumentParser(
        description="Offline tester for detect_object_real_task3.py HSV red-pillow detection"
    )
    parser.add_argument("--images", required=True,
                        help="Path to an image file or folder of images")
    parser.add_argument("--hue_low1",  type=int, default=0,
                        help="Lower red hue band — low  (default 0)")
    parser.add_argument("--hue_high1", type=int, default=10,
                        help="Lower red hue band — high (default 10)")
    parser.add_argument("--hue_low2",  type=int, default=170,
                        help="Upper red hue band — low  (default 170)")
    parser.add_argument("--hue_high2", type=int, default=180,
                        help="Upper red hue band — high (default 180)")
    parser.add_argument("--sat_min",    type=int, default=80,
                        help="Minimum HSV saturation 0-255 (default 80)")
    parser.add_argument("--val_min",    type=int, default=60,
                        help="Minimum HSV value/brightness 0-255 (default 60)")
    parser.add_argument("--min_area_px", type=int, default=500,
                        help="Minimum contour area in pixels (default 500)")
    parser.add_argument("--save_dir",   default=None,
                        help="Save annotated images to this folder")
    parser.add_argument("--no_display", action="store_true",
                        help="Don't open display windows (headless mode)")
    parser.add_argument("--show_mask",  action="store_true",
                        help="Also show the binary HSV mask (useful for tuning)")
    args = parser.parse_args()

    images = find_images(args.images)

    if args.save_dir:
        Path(args.save_dir).mkdir(parents=True, exist_ok=True)

    print("=" * 58)
    print("detect_object_real_task3.py  —  OFFLINE HSV TESTER")
    print("=" * 58)
    print(f"Images     : {len(images)}")
    print(f"Red masks  : hue [{args.hue_low1}–{args.hue_high1}] OR [{args.hue_low2}–{args.hue_high2}]")
    print(f"Saturation : >= {args.sat_min}")
    print(f"Value      : >= {args.val_min}")
    print(f"Min area   : {args.min_area_px} px")
    print()

    detected_count = 0

    for img_path in images:
        bgr = cv2.imread(str(img_path))
        if bgr is None:
            print(f"  WARNING: Could not read {img_path.name} — skipping")
            continue

        detection, mask = detect_target(
            bgr,
            args.hue_low1, args.hue_high1,
            args.hue_low2, args.hue_high2,
            args.sat_min, args.val_min,
            args.min_area_px,
        )

        print_topic_simulation(detection, img_path.name)

        if detection is not None:
            detected_count += 1

        vis = annotate(bgr, detection)

        if args.save_dir:
            out = Path(args.save_dir) / img_path.name
            cv2.imwrite(str(out), vis)
            print(f"  Saved annotated image → {out}")
            if args.show_mask:
                mask_out = Path(args.save_dir) / f"mask_{img_path.name}"
                cv2.imwrite(str(mask_out), mask)
                print(f"  Saved mask            → {mask_out}")

        if not args.no_display:
            show_image(f"Detection — {img_path.name}", vis)
            if args.show_mask:
                show_image(f"HSV mask  — {img_path.name}", mask)
            key = cv2.waitKey(0)
            cv2.destroyAllWindows()
            if key == 27:  # ESC
                print("\nStopped by user.")
                break

    # Summary
    total = len(images)
    print()
    print("=" * 58)
    print("SUMMARY")
    print("=" * 58)
    print(f"Tested  : {total} image(s)")
    print(f"Detected: {detected_count}/{total}  ({100*detected_count//max(total,1)}%)")
    print()
    if detected_count == total:
        print("All images detected — HSV thresholds look good.")
    elif detected_count == 0:
        print("Nothing detected. Try:")
        print("  --hue_high1 15          (widen lower red band)")
        print("  --hue_low2 165          (widen upper red band)")
        print("  --sat_min 50            (lower saturation threshold)")
        print("  --val_min 40            (lower brightness threshold)")
        print("  --min_area_px 200       (smaller minimum blob size)")
        print("  --show_mask             (visualise the raw mask to debug)")
    else:
        print(f"Partial detection ({detected_count}/{total}). Try widening thresholds slightly.")


if __name__ == "__main__":
    main()
