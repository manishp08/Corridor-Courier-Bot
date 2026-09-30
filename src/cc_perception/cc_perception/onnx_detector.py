"""YOLOv8-format ONNX detector (onnxruntime, CPU by default). No ROS imports.

Output tensor layout (Ultralytics export): (1, 4 + num_classes, N) with
rows cx, cy, w, h in letterboxed pixels followed by per-class scores.

Class mapping: a model fine-tuned on the project classes should list them
as [person, cart, low_obstacle, glass]. With stock COCO weights only
"person" (and a few proxies) are mapped; carts, pallets and glass need
fine-tuning, which is why the Gazebo pipeline also offers the oracle backend.
"""
import time

import numpy as np

COCO_TO_PROJECT = {
    0: "person",
    # COCO has no cart/pallet/glass; these proxies are better than nothing.
    24: "low_obstacle",   # backpack
    26: "low_obstacle",   # handbag
    28: "low_obstacle",   # suitcase
    13: "cart",           # bench
    56: "cart",           # chair
}
PROJECT_CLASSES = ["person", "cart", "low_obstacle", "glass"]


def letterbox(img, size=640):
    h, w = img.shape[:2]
    r = min(size / h, size / w)
    nh, nw = int(round(h * r)), int(round(w * r))
    import cv2
    resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    out = np.full((size, size, 3), 114, np.uint8)
    top, left = (size - nh) // 2, (size - nw) // 2
    out[top:top + nh, left:left + nw] = resized
    return out, r, left, top


def nms(boxes, scores, iou=0.45):
    order = np.argsort(-scores)
    keep = []
    while len(order):
        i = order[0]
        keep.append(i)
        if len(order) == 1:
            break
        xx1 = np.maximum(boxes[i, 0], boxes[order[1:], 0])
        yy1 = np.maximum(boxes[i, 1], boxes[order[1:], 1])
        xx2 = np.minimum(boxes[i, 2], boxes[order[1:], 2])
        yy2 = np.minimum(boxes[i, 3], boxes[order[1:], 3])
        inter = np.clip(xx2 - xx1, 0, None) * np.clip(yy2 - yy1, 0, None)
        area = lambda b: (b[..., 2] - b[..., 0]) * (b[..., 3] - b[..., 1])  # noqa: E731
        ov = inter / (area(boxes[i]) + area(boxes[order[1:]]) - inter + 1e-9)
        order = order[1:][ov < iou]
    return np.array(keep, int)


def decode(output, ratio, pad_x, pad_y, class_map, conf=0.35, iou=0.45):
    """Raw (1, 4+C, N) output -> list of (cls, score, x1, y1, x2, y2) in the original image."""
    pred = output[0].T                       # (N, 4 + C)
    scores_all = pred[:, 4:]
    cls_id = scores_all.argmax(axis=1)
    score = scores_all[np.arange(len(pred)), cls_id]
    mask = (score >= conf) & np.isin(cls_id, list(class_map))
    if not mask.any():
        return []
    b, s, c = pred[mask, :4], score[mask], cls_id[mask]
    xyxy = np.column_stack([b[:, 0] - b[:, 2] / 2, b[:, 1] - b[:, 3] / 2,
                            b[:, 0] + b[:, 2] / 2, b[:, 1] + b[:, 3] / 2])
    xyxy[:, [0, 2]] = (xyxy[:, [0, 2]] - pad_x) / ratio
    xyxy[:, [1, 3]] = (xyxy[:, [1, 3]] - pad_y) / ratio
    out = []
    for k in np.unique(c):                   # class-wise NMS
        idx = np.flatnonzero(c == k)
        for j in idx[nms(xyxy[idx], s[idx], iou)]:
            out.append((class_map[int(k)], float(s[j]), *map(float, xyxy[j])))
    return out


class OnnxDetector:
    def __init__(self, model_path, input_size=640, conf=0.35, iou=0.45, class_map=None, threads=0):
        import onnxruntime as ort
        opts = ort.SessionOptions()
        if threads:
            opts.intra_op_num_threads = threads
        self.session = ort.InferenceSession(model_path, opts, providers=ort.get_available_providers())
        self.input_name = self.session.get_inputs()[0].name
        self.size = input_size
        self.conf, self.iou = conf, iou
        n_out = self.session.get_outputs()[0].shape[1]
        if class_map is None:
            class_map = ({i: c for i, c in enumerate(PROJECT_CLASSES)} if n_out == 4 + len(PROJECT_CLASSES)
                         else COCO_TO_PROJECT)
        self.class_map = class_map
        self.last_latency_ms = float("nan")

    def __call__(self, rgb):
        t0 = time.perf_counter()
        img, r, px, py = letterbox(rgb, self.size)
        x = np.ascontiguousarray(img.transpose(2, 0, 1)[None], dtype=np.float32) / 255.0
        out = self.session.run(None, {self.input_name: x})[0]
        dets = decode(out, r, px, py, self.class_map, self.conf, self.iou)
        self.last_latency_ms = (time.perf_counter() - t0) * 1e3
        return dets
