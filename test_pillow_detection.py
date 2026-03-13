#!/usr/bin/env python3
"""
Offline YOLO pillow detection tester.

Tests whether YOLO can reliably detect a pillow in iPhone images,
before committing to it as a perception strategy for Task 3.

No ROS needed — just Python + ultralytics.

Install:
    pip install ultralytics opencv-python

Usage:
    # Test a folder of images:
    python3 test_pillow_detection.py --images /path/to/iphone/photos/

    # Test a single image:
    python3 test_pillow_detection.py --images /path/to/photo.jpg

    # Try a larger model (more accurate, slower):
    python3 test_pillow_detection.py --images /path/to/photos/ --model yolov8s.pt

    # Lower confidence threshold to catch weak detections:
    python3 test_pillow_detection.py --images /path/to/photos/ --conf 0.2

    # Save annotated images to a folder:
    python3 test_pillow_detection.py --images /path/to/photos/ --save_dir ./results/
"""

import argparse
import os
import sys
from pathlib import Path

import cv2
import numpy as np

# COCO classes that could plausibly match a pillow
PILLOW_ADJACENT_CLASSES = {
    "pillow", "cushion", "couch", "sofa", "bed",
    "teddy bear", "blanket", "chair",
}

IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".webp", ".heic", ".heif"}


def load_model(model_path: str, conf_threshold: float):
    try:
        from ultralytics import YOLO
    except ImportError:
        print("ERROR: ultralytics not installed. Run: pip install ultralytics")
        sys.exit(1)

    print(f"Loading model: {model_path}")
    model = YOLO(model_path)
    print(f"Model loaded. Classes: {len(model.names)}")
    print()
    return model


def find_images(input_path: str):
    p = Path(input_path)
    if p.is_file():
        return [p]
    images = [
        f for f in sorted(p.iterdir())
        if f.suffix.lower() in IMAGE_EXTENSIONS
    ]
    if not images:
        print(f"No images found in {input_path}")
        sys.exit(1)
    return images


def run_detection(model, image_path: Path, conf_threshold: float):
    img = cv2.imread(str(image_path))
    if img is None:
        print(f"  WARNING: Could not read {image_path.name} — skipping")
        return None, None

    results = model.predict(source=img, conf=conf_threshold, verbose=False)
    return img, results[0]


def annotate_image(img, result, highlight_classes=None):
    vis = img.copy()
    boxes = getattr(result, "boxes", None)
    if boxes is None or len(boxes) == 0:
        return vis

    for xyxy, conf, cls_id in zip(
        boxes.xyxy.cpu().numpy(),
        boxes.conf.cpu().numpy(),
        boxes.cls.cpu().numpy(),
    ):
        x1, y1, x2, y2 = map(int, xyxy)
        cls_name = result.names[int(cls_id)]
        is_highlight = highlight_classes and cls_name.lower() in highlight_classes

        color = (0, 220, 0) if is_highlight else (180, 180, 180)
        thickness = 3 if is_highlight else 1

        cv2.rectangle(vis, (x1, y1), (x2, y2), color, thickness)
        label = f"{cls_name} {conf:.2f}"
        cv2.putText(vis, label, (x1, max(25, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

    return vis


def print_detections(result, image_name: str, highlight_classes=None):
    boxes = getattr(result, "boxes", None)

    print(f"  {image_name}")
    if boxes is None or len(boxes) == 0:
        print("    No detections above confidence threshold")
        return []

    detections = []
    for conf, cls_id in zip(
        boxes.conf.cpu().numpy(),
        boxes.cls.cpu().numpy(),
    ):
        cls_name = result.names[int(cls_id)]
        flag = " <-- PILLOW CANDIDATE" if (highlight_classes and cls_name.lower() in highlight_classes) else ""
        print(f"    {cls_name:<20} conf={conf:.3f}{flag}")
        detections.append((cls_name, float(conf)))

    return detections


def main():
    parser = argparse.ArgumentParser(description="Offline YOLO pillow detection tester")
    parser.add_argument("--images", required=True, help="Path to image file or folder of images")
    parser.add_argument("--model", default="yolov8n.pt", help="YOLO model to use (default: yolov8n.pt). Try yolov8s.pt or yolov8m.pt for better accuracy.")
    parser.add_argument("--conf", type=float, default=0.25, help="Confidence threshold (default: 0.25). Lower = catch more detections.")
    parser.add_argument("--save_dir", default=None, help="Save annotated images to this folder")
    parser.add_argument("--no_display", action="store_true", help="Don't display images (useful on headless machines)")
    args = parser.parse_args()

    model = load_model(args.model, args.conf)
    images = find_images(args.images)

    if args.save_dir:
        Path(args.save_dir).mkdir(parents=True, exist_ok=True)

    print("=" * 60)
    print(f"Testing {len(images)} image(s) | model={args.model} | conf>={args.conf}")
    print(f"Highlighting classes: {sorted(PILLOW_ADJACENT_CLASSES)}")
    print("=" * 60)
    print()

    # Per-image results
    all_detections = []         # list of (image_name, cls_name, conf)
    images_with_pillow = []
    images_with_nothing = []

    for image_path in images:
        img, result = run_detection(model, image_path, args.conf)
        if img is None:
            continue

        dets = print_detections(result, image_path.name, PILLOW_ADJACENT_CLASSES)

        pillow_dets = [(n, c) for n, c in dets if n.lower() in PILLOW_ADJACENT_CLASSES]
        if pillow_dets:
            images_with_pillow.append(image_path.name)
        if not dets:
            images_with_nothing.append(image_path.name)

        for cls_name, conf in dets:
            all_detections.append((image_path.name, cls_name, conf))

        # Annotate and show/save
        vis = annotate_image(img, result, PILLOW_ADJACENT_CLASSES)

        if args.save_dir:
            out_path = Path(args.save_dir) / image_path.name
            cv2.imwrite(str(out_path), vis)

        if not args.no_display:
            # Resize for display if image is large
            h, w = vis.shape[:2]
            if w > 1280:
                scale = 1280 / w
                vis = cv2.resize(vis, (int(w * scale), int(h * scale)))
            cv2.imshow(f"YOLO — {image_path.name}", vis)
            key = cv2.waitKey(0)
            cv2.destroyAllWindows()
            if key == 27:  # ESC to quit early
                print("\nStopped by user.")
                break

    # Summary report
    print()
    print("=" * 60)
    print("SUMMARY REPORT")
    print("=" * 60)
    print(f"Total images tested : {len(images)}")
    print(f"Pillow candidate found : {len(images_with_pillow)}/{len(images)} "
          f"({100*len(images_with_pillow)/max(len(images),1):.0f}%)")
    print(f"No detections at all   : {len(images_with_nothing)}/{len(images)}")
    print()

    if images_with_pillow:
        print("Images where pillow was detected:")
        for name in images_with_pillow:
            relevant = [(c, conf) for img_n, c, conf in all_detections
                        if img_n == name and c.lower() in PILLOW_ADJACENT_CLASSES]
            for cls_name, conf in relevant:
                print(f"  {name:<40} -> '{cls_name}' conf={conf:.3f}")
    else:
        print("Pillow was NOT detected in any image.")
        print("Consider:")
        print("  1. Try a larger model: --model yolov8m.pt")
        print("  2. Lower confidence: --conf 0.15")
        print("  3. Fine-tune YOLO on pillow images (see Roboflow)")
        print("  4. Use SAM3 (already on team GPU server) — text-prompted, no retraining needed")

    print()

    # Per-class summary across all images
    if all_detections:
        from collections import Counter
        class_counts = Counter(c for _, c, _ in all_detections)
        class_conf = {}
        for _, c, conf in all_detections:
            class_conf.setdefault(c, []).append(conf)

        print("All detected classes across all images:")
        print(f"  {'Class':<25} {'Count':>6}  {'Avg conf':>10}  {'Min':>6}  {'Max':>6}")
        print(f"  {'-'*25} {'-'*6}  {'-'*10}  {'-'*6}  {'-'*6}")
        for cls_name, count in class_counts.most_common():
            confs = class_conf[cls_name]
            flag = "  <-- PILLOW CANDIDATE" if cls_name.lower() in PILLOW_ADJACENT_CLASSES else ""
            print(f"  {cls_name:<25} {count:>6}  {np.mean(confs):>10.3f}  "
                  f"{min(confs):>6.3f}  {max(confs):>6.3f}{flag}")

    print()
    print("RECOMMENDATION:")
    detection_rate = len(images_with_pillow) / max(len(images), 1)
    if detection_rate >= 0.8:
        print("  Detection rate >= 80% — YOLO should work reliably for Task 3.")
    elif detection_rate >= 0.5:
        print("  Detection rate 50-80% — might be usable but consider a larger model or fine-tuning.")
    else:
        print("  Detection rate < 50% — YOLO alone is not reliable enough.")
        print("  Recommended alternative: use SAM3 (text-prompted segmentation) via the team GPU server.")
        print("  SAM3 is already running at 100.77.113.90 — no retraining needed.")


if __name__ == "__main__":
    main()
