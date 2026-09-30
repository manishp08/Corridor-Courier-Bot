"""Pure geometry: 2D detections + depth -> 3D object footprints (no ROS imports).

Frames: the camera optical frame (x right, y down, z forward) as published by
the RGB-D camera. The costmap layer transforms results into map/odom via TF.
"""
from dataclasses import dataclass

import numpy as np


@dataclass
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float

    @classmethod
    def from_k(cls, k):
        return cls(k[0], k[4], k[2], k[5])


@dataclass
class Detection2D:
    cls: str
    score: float
    x1: float
    y1: float
    x2: float
    y2: float


@dataclass
class Object3D:
    cls: str
    score: float
    x: float          # centre, camera optical frame (m)
    y: float
    z: float
    size_x: float     # footprint in the ground plane (m)
    size_y: float
    method: str       # "depth" or "ground_plane"


def robust_depth(depth, det, inner=0.5, min_valid=0.2):
    """Median of valid depth inside the central `inner` fraction of the box.
    Returns None when too few pixels are valid (glass, black surfaces)."""
    h, w = depth.shape
    cxb, cyb = (det.x1 + det.x2) / 2, (det.y1 + det.y2) / 2
    hw, hh = (det.x2 - det.x1) * inner / 2, (det.y2 - det.y1) * inner / 2
    u0, u1 = int(max(0, cxb - hw)), int(min(w, cxb + hw + 1))
    v0, v1 = int(max(0, cyb - hh)), int(min(h, cyb + hh + 1))
    if u1 <= u0 or v1 <= v0:
        return None
    patch = depth[v0:v1, u0:u1].astype(float)
    valid = np.isfinite(patch) & (patch > 0.1) & (patch < 20.0)
    if valid.mean() < min_valid:
        return None
    return float(np.median(patch[valid]))


def ray(intr, u, v):
    return np.array([(u - intr.cx) / intr.fx, (v - intr.cy) / intr.fy, 1.0])


def ground_plane_point(intr, u, v, cam_height, cam_pitch):
    """Intersect the ray through pixel (u, v) with the floor.

    cam_height: optical centre height above the floor (m); cam_pitch: downward
    tilt (rad). Returns the point in the optical frame, or None if the ray
    does not hit the floor in front of the camera."""
    d = ray(intr, u, v)
    # Optical frame: y points down. Rotate by pitch about x to a level frame.
    c, s = np.cos(cam_pitch), np.sin(cam_pitch)
    down = s * d[2] + c * d[1]           # component along gravity
    if down <= 1e-6:
        return None
    t = cam_height / down
    return d * t


def project(det, depth, intr, cam_height, cam_pitch, depth_is_to_surface=True):
    """3D footprint of a detection. Depth path for normal objects; ground-plane
    path (bottom edge of the box on the floor) when depth is invalid, which is
    how glass gets a position at all."""
    z = robust_depth(depth, det) if depth is not None else None
    width_px = det.x2 - det.x1
    if z is not None:
        u, v = (det.x1 + det.x2) / 2, (det.y1 + det.y2) / 2
        p = ray(intr, u, v) * z
        width = width_px * z / intr.fx
        if depth_is_to_surface:           # depth sees the front face; move to the centre
            p = p + np.array([0.0, 0.0, width / 2])
        return Object3D(det.cls, det.score, *p, width, width, "depth")
    p = ground_plane_point(intr, (det.x1 + det.x2) / 2, det.y2, cam_height, cam_pitch)
    if p is None:
        return None
    width = width_px * p[2] / intr.fx
    thickness = 0.05 if det.cls == "glass" else width
    return Object3D(det.cls, det.score, *p, width, thickness, "ground_plane")
