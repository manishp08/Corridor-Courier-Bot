"""Sensor models: 2D LiDAR, RGB-D detector, wheel encoders, gyro."""
from dataclasses import dataclass

import numpy as np

from .geometry import ray_circles, ray_segments, segments_block, wrap

CLASS_IDS = {"person": 0, "cart": 1, "low_obstacle": 2, "glass": 3, "unknown": 4}


@dataclass
class LidarParams:
    n_beams: int = 180
    min_range: float = 0.12
    max_range: float = 8.0
    sigma: float = 0.01
    # Glass returns specularly: probability a beam comes back falls off
    # with the incidence angle (deg) as peak * exp(-(inc / width)^2).
    glass_peak: float = 0.8
    glass_width_deg: float = 10.0


class Lidar:
    def __init__(self, params=None):
        self.p = params or LidarParams()
        self.angles = np.linspace(-np.pi, np.pi, self.p.n_beams, endpoint=False)

    def scan(self, world, pose, actor_states, rng):
        ang = pose[2] + self.angles
        ox, oy = pose[0], pose[1]
        opaque = world.lidar_opaque_segments()
        r = ray_segments(ox, oy, ang, opaque).min(axis=1)
        circles = world.lidar_static_circles()
        if len(actor_states):
            people = np.column_stack([actor_states[:, 0], actor_states[:, 1],
                                      np.full(len(actor_states), world.actors[0].lidar_radius)])
            circles = np.vstack([circles, people]) if len(circles) else people
        if len(circles):
            r = np.minimum(r, ray_circles(ox, oy, ang, circles).min(axis=1))
        if len(world.glass):
            rg = ray_segments(ox, oy, ang, world.glass)
            g = world.glass
            ex, ey = g[:, 2] - g[:, 0], g[:, 3] - g[:, 1]
            n = np.hypot(ex, ey)
            nx, ny = -ey / n, ex / n
            cos_inc = np.abs(np.cos(ang)[:, None] * nx + np.sin(ang)[:, None] * ny)
            inc = np.degrees(np.arccos(np.clip(cos_inc, 0.0, 1.0)))
            p_ret = self.p.glass_peak * np.exp(-(inc / self.p.glass_width_deg) ** 2)
            returned = rng.random(rg.shape) < p_ret
            rg = np.where(returned, rg, np.inf)
            r = np.minimum(r, rg.min(axis=1))
        r = r + rng.normal(0.0, self.p.sigma, r.shape)
        r[(r > self.p.max_range) | (r < self.p.min_range)] = np.inf
        return self.angles, r


@dataclass
class CameraParams:
    hfov_deg: float = 87.0          # RealSense D435-class
    mount_x: float = 0.20           # forward offset from base_link
    min_range_floor: float = 0.5    # floor objects closer than this are below the vertical FOV
    min_range: float = 0.3
    max_range: float = 6.0
    glass_max_range: float = 4.0
    p_detect: tuple = (("person", 0.95), ("cart", 0.90), ("low_obstacle", 0.85), ("glass", 0.60))
    range_falloff: float = 0.3      # detection probability lost at max range (linear)
    sigma_depth: tuple = (0.03, 0.01)     # sigma = a + b z^2 (stereo depth)
    sigma_ground: tuple = (0.05, 0.02)    # ground-plane projection when depth is invalid (glass)
    sigma_lateral: float = 0.02
    false_positive_rate: float = 0.03     # ghost detections per frame
    latency: float = 0.1                  # capture -> costmap (s)


class CameraDetector:
    """Stand-in for detector_node: what a YOLO + depth pipeline delivers,
    with range-dependent recall, depth noise, occlusion, FOV and latency.
    Returns detections in the camera frame (range, bearing) plus class and
    footprint so the costmap side has to use the robot's belief pose."""

    def __init__(self, params=None):
        self.p = params or CameraParams()
        self.p_det = dict(self.p.p_detect)
        self.half_fov = np.deg2rad(self.p.hfov_deg) / 2.0

    def _cam_pose(self, pose):
        return np.array([pose[0] + self.p.mount_x * np.cos(pose[2]),
                         pose[1] + self.p.mount_x * np.sin(pose[2]), pose[2]])

    def _in_view(self, cam, x, y, max_range, min_range):
        dx, dy = x - cam[0], y - cam[1]
        z = np.hypot(dx, dy)
        bearing = wrap(np.arctan2(dy, dx) - cam[2])
        return (min_range <= z <= max_range) and abs(bearing) <= self.half_fov, z, bearing

    def _keep(self, cls, z, max_range, rng):
        p = self.p_det[cls] * (1.0 - self.p.range_falloff * z / max_range)
        return rng.random() < p

    def _noisy(self, z, bearing, sig, rng):
        zn = z + rng.normal(0.0, sig[0] + sig[1] * z * z)
        bn = bearing + rng.normal(0.0, self.p.sigma_lateral / max(z, 0.3))
        return zn, bn

    def detect(self, world, pose, actor_states, rng):
        """List of (cls, range, bearing, rel_yaw, size_x, size_y) in the camera frame."""
        cam = self._cam_pose(pose)
        block = world.camera_blocking_segments()
        dets = []
        for a in actor_states:
            ok, z, b = self._in_view(cam, a[0], a[1], self.p.max_range, self.p.min_range)
            if ok and not segments_block(cam[:2], a[:2], block) and self._keep("person", z, self.p.max_range, rng):
                zn, bn = self._noisy(z, b, self.p.sigma_depth, rng)
                dets.append(("person", zn, bn, 0.0, 0.5, 0.5))
        for o in world.obstacles:
            if o.cls not in self.p_det:
                continue
            min_r = self.p.min_range_floor if o.height < 0.3 else self.p.min_range
            ok, z, b = self._in_view(cam, o.x, o.y, self.p.max_range, min_r)
            if not ok or segments_block(cam[:2], (o.x, o.y), block):
                continue
            if self._keep(o.cls, z, self.p.max_range, rng):
                zn, bn = self._noisy(z, b, self.p.sigma_depth, rng)
                side = max(o.sx, o.sy) * rng.uniform(0.9, 1.1)   # square footprint from bbox width
                dets.append((o.cls, zn, bn, 0.0, side, side))
        for g in world.glass:
            d = self._visible_glass(cam, g, block)
            if d is None:
                continue
            (mx, my), length, seg_yaw = d
            ok, z, b = self._in_view(cam, mx, my, self.p.glass_max_range, self.p.min_range)
            if ok and self._keep("glass", z, self.p.glass_max_range, rng):
                zn, bn = self._noisy(z, b, self.p.sigma_ground, rng)
                dets.append(("glass", zn, bn, wrap(seg_yaw - cam[2]), length, 0.05))
        n_fp = rng.poisson(self.p.false_positive_rate)
        for _ in range(n_fp):
            dets.append(("low_obstacle", rng.uniform(1.0, 5.0), rng.uniform(-self.half_fov, self.half_fov),
                         0.0, 0.4, 0.4))
        return cam, dets

    def _visible_glass(self, cam, g, block):
        """Mid-point, length and yaw of the part of a glass pane inside the FOV and range."""
        ts = np.linspace(0.0, 1.0, 41)
        xs = g[0] + ts * (g[2] - g[0])
        ys = g[1] + ts * (g[3] - g[1])
        dx, dy = xs - cam[0], ys - cam[1]
        z = np.hypot(dx, dy)
        bearing = np.abs(wrap(np.arctan2(dy, dx) - cam[2]))
        vis = (z <= self.p.glass_max_range) & (z >= self.p.min_range) & (bearing <= self.half_fov)
        if vis.sum() < 2:
            return None
        idx = np.flatnonzero(vis)
        pts = np.column_stack([xs[idx], ys[idx]])
        mid = pts.mean(axis=0)
        if segments_block(cam[:2], mid, block):
            return None
        length = float(np.hypot(*(pts[-1] - pts[0])))
        return mid, max(length, 0.1), float(np.arctan2(g[3] - g[1], g[2] - g[0]))


@dataclass
class OdomParams:
    # Per-run systematic errors (calibration / effective wheelbase on a polished floor).
    scale_v_sigma: float = 0.01
    yaw_scale_mean: float = 0.04    # turning scrub makes encoders over-report rotation
    yaw_scale_sigma: float = 0.02
    sigma_v: float = 0.005
    sigma_w: float = 0.01
    # Slip bursts (wheel spin / skid on polished floor).
    slip_prob_per_s: float = 0.1
    slip_duration: tuple = (0.3, 1.0)
    slip_w_error: float = 0.15
    slip_v_gain: tuple = (1.0, 1.15)


class WheelEncoders:
    def __init__(self, rng, params=None):
        self.p = params or OdomParams()
        self.scale_v = 1.0 + rng.normal(0.0, self.p.scale_v_sigma)
        self.scale_w = 1.0 + rng.normal(self.p.yaw_scale_mean, self.p.yaw_scale_sigma)
        self.slip_left = 0.0
        self.slip_w = 0.0
        self.slip_v = 1.0

    def measure(self, v, w, dt, rng):
        if self.slip_left <= 0.0 and rng.random() < self.p.slip_prob_per_s * dt and (abs(v) > 0.05 or abs(w) > 0.1):
            self.slip_left = rng.uniform(*self.p.slip_duration)
            self.slip_w = rng.uniform(-self.p.slip_w_error, self.p.slip_w_error)
            self.slip_v = rng.uniform(*self.p.slip_v_gain)
        vm = v * self.scale_v + rng.normal(0.0, self.p.sigma_v)
        wm = w * self.scale_w + rng.normal(0.0, self.p.sigma_w)
        if self.slip_left > 0.0:
            vm *= self.slip_v
            wm += self.slip_w
            self.slip_left -= dt
        return vm, wm


@dataclass
class ImuParams:
    sigma: float = 0.003            # rad/s white noise
    bias_sigma: float = 0.001       # residual bias after start-up calibration
    bias_walk: float = 0.0002       # rad/s/sqrt(s)


class Gyro:
    def __init__(self, rng, params=None):
        self.p = params or ImuParams()
        self.bias = rng.normal(0.0, self.p.bias_sigma)

    def measure(self, w, dt, rng):
        self.bias += rng.normal(0.0, self.p.bias_walk * np.sqrt(dt))
        return w + self.bias + rng.normal(0.0, self.p.sigma)
