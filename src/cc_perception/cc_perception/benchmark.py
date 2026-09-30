"""Perception latency benchmark (no ROS): ONNX inference + depth projection per frame.

  python -m cc_perception.benchmark --model models/yolov8n.onnx --frames 200 --out results/perception_latency.md
"""
import argparse
import platform
import time

import numpy as np

from .onnx_detector import OnnxDetector
from .projection import Detection2D, Intrinsics, project


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--model", required=True)
    p.add_argument("--frames", type=int, default=200)
    p.add_argument("--threads", type=int, default=0)
    p.add_argument("--size", type=int, default=640)
    p.add_argument("--image", default="", help="optional RGB image to run on (else a synthetic frame)")
    p.add_argument("--out", default="")
    a = p.parse_args()
    det = OnnxDetector(a.model, input_size=a.size, threads=a.threads, conf=0.25)
    if a.image:
        import cv2
        rgb = cv2.cvtColor(cv2.imread(a.image), cv2.COLOR_BGR2RGB)
    else:
        rgb = (np.random.default_rng(0).random((480, 640, 3)) * 255).astype(np.uint8)
    depth = np.full(rgb.shape[:2], 3.0, np.float32)
    intr = Intrinsics(fx=rgb.shape[1] / (2 * np.tan(1.5184 / 2)), fy=rgb.shape[1] / (2 * np.tan(1.5184 / 2)),
                      cx=rgb.shape[1] / 2, cy=rgb.shape[0] / 2)
    for _ in range(10):
        det(rgb)
    infer, proj, total, n_det = [], [], [], []
    for _ in range(a.frames):
        t0 = time.perf_counter()
        dets = det(rgb)
        t1 = time.perf_counter()
        for c, s, x1, y1, x2, y2 in dets:
            project(Detection2D(c, s, x1, y1, x2, y2), depth, intr, 0.45, 0.2618)
        t2 = time.perf_counter()
        infer.append((t1 - t0) * 1e3)
        proj.append((t2 - t1) * 1e3)
        total.append((t2 - t0) * 1e3)
        n_det.append(len(dets))
    q = lambda x, k: float(np.percentile(x, k))  # noqa: E731
    lines = [
        "## Perception latency (ONNX detector, CPU)\n",
        f"Model `{a.model.split('/')[-1]}`, input {a.size}×{a.size}, {a.frames} frames of "
        f"{rgb.shape[1]}×{rgb.shape[0]}, onnxruntime on `{platform.processor() or platform.machine()}` "
        f"({'default' if not a.threads else a.threads} threads). "
        f"Input: {'`' + a.image + '`' if a.image else 'synthetic noise frame'}.\n",
        "| Stage | Median (ms) | p95 (ms) |",
        "|---|---|---|",
        f"| Letterbox + inference + NMS | {q(infer, 50):.1f} | {q(infer, 95):.1f} |",
        f"| Depth projection (all detections) | {q(proj, 50):.3f} | {q(proj, 95):.3f} |",
        f"| Total per frame | {q(total, 50):.1f} | {q(total, 95):.1f} |",
        "",
        f"Sustainable rate on this machine: ~{1000 / q(total, 50):.0f} Hz (camera runs at 15 Hz).\n",
    ]
    text = "\n".join(lines)
    print(text)
    if a.out:
        with open(a.out, "w") as f:
            f.write(text)


if __name__ == "__main__":
    main()
