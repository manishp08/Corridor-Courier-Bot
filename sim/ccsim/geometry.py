"""Vectorised 2D geometry helpers (rays, segments, circles, oriented boxes)."""
import numpy as np


def wrap(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def compose(a, b):
    """SE(2) composition a (+) b, poses as (x, y, yaw)."""
    c, s = np.cos(a[2]), np.sin(a[2])
    return np.array([a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2])])


def inverse(a):
    c, s = np.cos(a[2]), np.sin(a[2])
    return np.array([-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2]])


def ray_segments(ox, oy, angles, segs):
    """Distance along each ray to each segment. Returns (B, N) with inf for misses."""
    if len(segs) == 0:
        return np.full((len(angles), 0), np.inf)
    dx = np.cos(angles)[:, None]
    dy = np.sin(angles)[:, None]
    x1, y1, x2, y2 = segs[:, 0], segs[:, 1], segs[:, 2], segs[:, 3]
    ex, ey = x2 - x1, y2 - y1
    wx, wy = x1 - ox, y1 - oy
    denom = dx * ey - dy * ex
    with np.errstate(divide="ignore", invalid="ignore"):
        t = (wx * ey - wy * ex) / denom
        u = (wx * dy - wy * dx) / denom
    ok = (np.abs(denom) > 1e-12) & (t > 1e-6) & (u >= 0.0) & (u <= 1.0)
    return np.where(ok, t, np.inf)


def ray_circles(ox, oy, angles, circles):
    """Distance along each ray to each circle (x, y, r). Returns (B, M)."""
    if len(circles) == 0:
        return np.full((len(angles), 0), np.inf)
    dx = np.cos(angles)[:, None]
    dy = np.sin(angles)[:, None]
    cx = circles[:, 0] - ox
    cy = circles[:, 1] - oy
    r = circles[:, 2]
    b = dx * cx + dy * cy
    c = cx * cx + cy * cy - r * r
    disc = b * b - c
    with np.errstate(invalid="ignore"):
        t = b - np.sqrt(disc)
    ok = (disc >= 0.0) & (t > 1e-6)
    return np.where(ok, t, np.inf)


def point_segment_distance(px, py, segs):
    """Distance from point to each segment, (N,)."""
    if len(segs) == 0:
        return np.full(0, np.inf)
    x1, y1, x2, y2 = segs[:, 0], segs[:, 1], segs[:, 2], segs[:, 3]
    ex, ey = x2 - x1, y2 - y1
    l2 = np.maximum(ex * ex + ey * ey, 1e-12)
    t = np.clip(((px - x1) * ex + (py - y1) * ey) / l2, 0.0, 1.0)
    return np.hypot(x1 + t * ex - px, y1 + t * ey - py)


def point_box_distance(px, py, boxes):
    """Distance from point to each oriented box (x, y, yaw, sx, sy); 0 inside."""
    if len(boxes) == 0:
        return np.full(0, np.inf)
    c, s = np.cos(boxes[:, 2]), np.sin(boxes[:, 2])
    dx, dy = px - boxes[:, 0], py - boxes[:, 1]
    lx = np.abs(c * dx + s * dy) - 0.5 * boxes[:, 3]
    ly = np.abs(-s * dx + c * dy) - 0.5 * boxes[:, 4]
    return np.hypot(np.maximum(lx, 0.0), np.maximum(ly, 0.0))


def box_segments(x, y, yaw, sx, sy):
    c, s = np.cos(yaw), np.sin(yaw)
    hx, hy = 0.5 * sx, 0.5 * sy
    corners = [(hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy)]
    pts = [(x + c * a - s * b, y + s * a + c * b) for a, b in corners]
    return np.array([[*pts[i], *pts[(i + 1) % 4]] for i in range(4)])


def segments_block(p0, p1, segs):
    """True if the open segment p0-p1 crosses any of segs (line of sight test)."""
    if len(segs) == 0:
        return False
    d = np.asarray(p1, float) - np.asarray(p0, float)
    length = float(np.hypot(*d))
    if length < 1e-9:
        return False
    ang = np.array([np.arctan2(d[1], d[0])])
    t = ray_segments(p0[0], p0[1], ang, segs)
    return bool(np.any(t < length - 1e-3))
