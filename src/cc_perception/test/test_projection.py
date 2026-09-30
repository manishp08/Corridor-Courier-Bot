import math

import numpy as np

from cc_perception.onnx_detector import decode, nms
from cc_perception.oracle import ActorScript, OracleDetector
from cc_perception.projection import Detection2D, Intrinsics, ground_plane_point, project

INTR = Intrinsics(fx=365.0, fy=365.0, cx=320.0, cy=240.0)


def test_depth_projection_centre_and_width():
    depth = np.full((480, 640), 3.0, np.float32)
    det = Detection2D("person", 0.9, 300, 140, 340, 340)          # 40 px wide, centred
    o = project(det, depth, INTR, 0.45, 0.26)
    assert o.method == "depth"
    width = 40 * 3.0 / 365.0
    assert abs(o.size_x - width) < 1e-6
    assert abs(o.z - (3.0 + width / 2)) < 1e-6                     # front face -> centre
    assert abs(o.x) < 0.01


def test_glass_falls_back_to_ground_plane():
    depth = np.full((480, 640), np.nan, np.float32)                # depth invalid on glass
    det = Detection2D("glass", 0.8, 100, 100, 540, 330)
    o = project(det, depth, INTR, 0.45, 0.0)
    assert o is not None and o.method == "ground_plane"
    # level camera 0.45 m up, bottom edge 90 px below centre -> z = 0.45 * fy / 90
    assert abs(o.z - 0.45 * 365.0 / 90.0) < 1e-6
    assert o.size_y == 0.05


def test_ground_plane_ray_above_horizon_is_none():
    assert ground_plane_point(INTR, 320, 100, 0.45, 0.0) is None


def test_decode_and_nms():
    out = np.zeros((1, 84, 3), np.float32)
    out[0, :4, 0] = [320, 320, 100, 200]; out[0, 4, 0] = 0.9       # person
    out[0, :4, 1] = [322, 318, 100, 200]; out[0, 4, 1] = 0.8       # duplicate person
    out[0, :4, 2] = [100, 100, 50, 50]; out[0, 4 + 5, 2] = 0.9     # bus: unmapped class
    dets = decode(out, 1.0, 0, 0, {0: "person"})
    assert len(dets) == 1 and dets[0][0] == "person"
    boxes = np.array([[0, 0, 10, 10], [1, 1, 10, 10], [20, 20, 30, 30]], float)
    assert sorted(nms(boxes, np.array([0.9, 0.8, 0.7]))) == [0, 2]


def test_actor_script_ping_pong():
    a = ActorScript({"waypoints": [[0, 0], [10, 0]], "speed": 1.0, "phase": 0.0})
    assert np.allclose(a.position(5.0), [5, 0])
    assert np.allclose(a.position(15.0), [5, 0])       # walking back


def test_oracle_sees_person_ahead_in_base_frame():
    m = {"objects": [], "glass": [], "actors": [{"waypoints": [[3, 0], [3.0001, 0]], "speed": 1e-6, "phase": 0}]}
    o = OracleDetector(m, seed=1)
    hits = [d for _ in range(50) for d in o.detect((0.0, 0.0, 0.0), 0.0)]
    assert len(hits) > 35
    assert all(abs(d["x"] - 3.0) < 0.3 and abs(d["y"]) < 0.1 for d in hits)
    behind = [d for _ in range(20) for d in o.detect((0.0, 0.0, math.pi), 0.0)]
    assert behind == []
