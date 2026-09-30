"""Ground-truth "oracle" detector for Gazebo runs without a fine-tuned model.

Reads the world manifest (objects, glass, actor scripts) and the robot's
ground-truth pose, and emits what a detector + depth pipeline would: objects
in the camera FOV with range-dependent recall and depth noise. The noise model
is the one used by the 2D surrogate (sim/ccsim/sensors.py CameraParams), so
the two simulators see statistically similar detections. No ROS imports.
"""
import math

import numpy as np

P_DETECT = {"person": 0.95, "cart": 0.90, "low_obstacle": 0.85, "glass": 0.60}


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class ActorScript:
    """Position of a looped ping-pong actor at sim time t (matches the SDF trajectory)."""

    def __init__(self, spec):
        self.wp = np.asarray(spec["waypoints"], float)
        self.speed = spec["speed"]
        self.length = float(np.hypot(*(self.wp[-1] - self.wp[0])))
        self.s0 = (spec["phase"] * self.speed) % (2 * self.length)

    def position(self, t):
        s = (self.s0 + self.speed * t) % (2 * self.length)
        if s > self.length:
            s = 2 * self.length - s
        return self.wp[0] + (self.wp[-1] - self.wp[0]) * s / max(self.length, 1e-9)


class OracleDetector:
    def __init__(self, manifest, hfov=1.5184, max_range=6.0, glass_range=4.0, min_range_floor=0.5,
                 sigma_depth=(0.03, 0.01), sigma_ground=(0.05, 0.02), seed=0):
        self.m = manifest
        self.actors = [ActorScript(a) for a in manifest["actors"]]
        self.half_fov = hfov / 2
        self.max_range, self.glass_range, self.min_floor = max_range, glass_range, min_range_floor
        self.sd, self.sg = sigma_depth, sigma_ground
        self.rng = np.random.default_rng(seed)

    def _emit(self, cam, cls, x, y, max_range, sx, sy, sig, min_range=0.3, yaw=0.0):
        dx, dy = x - cam[0], y - cam[1]
        z = math.hypot(dx, dy)
        b = wrap(math.atan2(dy, dx) - cam[2])
        if not (min_range <= z <= max_range and abs(b) <= self.half_fov):
            return None
        if self.rng.random() > P_DETECT[cls] * (1 - 0.3 * z / max_range):
            return None
        zn = z + self.rng.normal(0, sig[0] + sig[1] * z * z)
        bn = b + self.rng.normal(0, 0.02 / max(z, 0.3))
        return {"cls": cls, "score": 0.9, "range": zn, "bearing": bn, "yaw": wrap(yaw - cam[2]),
                "size_x": sx, "size_y": sy}

    def detect(self, robot_pose, t, mount_x=0.20):
        """robot_pose = true (x, y, yaw) of base_footprint in the world.

        Returns dicts with the object centre (x, y) and yaw in the *base_footprint*
        frame; the node stamps them with the capture time so the costmap layer
        places them using the robot's believed pose (TF), not ground truth."""
        cam_pose = (robot_pose[0] + mount_x * math.cos(robot_pose[2]),
                    robot_pose[1] + mount_x * math.sin(robot_pose[2]), robot_pose[2])
        out = self._detect_cam(cam_pose, t)
        for d in out:
            d["x"] = mount_x + d["range"] * math.cos(d["bearing"])
            d["y"] = d["range"] * math.sin(d["bearing"])
        return out

    def _detect_cam(self, cam_pose, t):
        out = []
        for a in self.actors:
            p = a.position(t)
            d = self._emit(cam_pose, "person", p[0], p[1], self.max_range, 0.5, 0.5, self.sd)
            if d:
                out.append(d)
        for o in self.m["objects"]:
            if o["class"] not in P_DETECT:
                continue
            side = max(o["size_x"], o["size_y"])
            mr = self.min_floor if o["height"] < 0.3 else 0.3
            d = self._emit(cam_pose, o["class"], o["x"], o["y"], self.max_range, side, side, self.sd, mr)
            if d:
                out.append(d)
        for g in self.m["glass"]:
            ts = np.linspace(0, 1, 41)
            xs, ys = g[0] + ts * (g[2] - g[0]), g[1] + ts * (g[3] - g[1])
            z = np.hypot(xs - cam_pose[0], ys - cam_pose[1])
            b = np.abs([wrap(math.atan2(y - cam_pose[1], x - cam_pose[0]) - cam_pose[2]) for x, y in zip(xs, ys)])
            vis = (z <= self.glass_range) & (b <= self.half_fov)
            if vis.sum() < 2:
                continue
            idx = np.flatnonzero(vis)
            mx, my = xs[idx].mean(), ys[idx].mean()
            length = math.hypot(xs[idx[-1]] - xs[idx[0]], ys[idx[-1]] - ys[idx[0]])
            d = self._emit(cam_pose, "glass", mx, my, self.glass_range, max(length, 0.1), 0.05, self.sg,
                           yaw=math.atan2(g[3] - g[1], g[2] - g[0]))
            if d:
                out.append(d)
        return out
