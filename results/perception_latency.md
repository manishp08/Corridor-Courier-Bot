## Perception latency (ONNX detector, CPU)

Model `yolov8n.onnx`, input 640×640, 200 frames of 810×1080, onnxruntime on `x86_64` (default threads). Input: `bus.jpg` (Ultralytics sample image).

| Stage | Median (ms) | p95 (ms) |
|---|---|---|
| Letterbox + inference + NMS | 75.6 | 94.5 |
| Depth projection (all detections) | 1.508 | 2.236 |
| Total per frame | 77.1 | 96.7 |

Sustainable rate on this machine: ~13 Hz (camera runs at 15 Hz).
