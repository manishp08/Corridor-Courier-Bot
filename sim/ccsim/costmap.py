"""Layered costmap: static (LiDAR SLAM map) + obstacle (LiDAR) + perception (C++) + inflation."""
from dataclasses import dataclass, field

import numpy as np
from scipy import ndimage

from .costlib import CostModel
from .sensors import CLASS_IDS
from .world import _rasterize_segments

LETHAL, INSCRIBED = 254, 253


@dataclass
class ClassCfg:
    enabled: bool = True
    inflation_radius: float = 0.55
    cost_scaling_factor: float = 3.0
    persistence: float = 2.0
    min_hits: int = 1
    association_gate: float = 0.4


# Mirrors cc_navigation/config/perception_layer_*.yaml.
STATIC_CLASSES = {
    "cart": ClassCfg(persistence=5.0, min_hits=2),
    "low_obstacle": ClassCfg(persistence=10.0, min_hits=2),
    "glass": ClassCfg(persistence=20.0, min_hits=2, association_gate=0.6),
}
PERSON_PLAIN = ClassCfg(inflation_radius=0.55, cost_scaling_factor=3.0, persistence=0.5, min_hits=1, association_gate=0.6)
PERSON_AWARE = ClassCfg(inflation_radius=1.2, cost_scaling_factor=1.5, persistence=0.5, min_hits=1, association_gate=0.6)


@dataclass
class CostmapParams:
    resolution: float = 0.05
    inscribed_radius: float = 0.25
    inflation_radius: float = 0.55
    cost_scaling_factor: float = 3.0
    obstacle_max_range: float = 6.0
    raytrace_max_range: float = 8.0
    use_camera: bool = False
    classes: dict = field(default_factory=dict)


class Costmap:
    def __init__(self, world, params):
        self.p = params
        x0, y0, x1, y1 = world.bounds
        self.origin = np.array([x0, y0])
        res = params.resolution
        self.shape = (int(np.ceil((y1 - y0) / res)), int(np.ceil((x1 - x0) / res)))
        # The map was built by LiDAR SLAM on the empty world: walls only.
        # Glass and floor-level clutter are absent, which is the point.
        self.static = np.zeros(self.shape, bool)
        _rasterize_segments(self.static, world.walls, x0, y0, res)
        self.obstacle = np.zeros(self.shape, bool)
        self.model = None
        if params.use_camera:
            self.model = CostModel(params.inscribed_radius)
            for name, cid in CLASS_IDS.items():
                c = params.classes.get(name, ClassCfg(enabled=False))
                self.model.set_class(cid, c.enabled, c.inflation_radius, c.cost_scaling_factor,
                                     c.persistence, c.min_hits, c.association_gate)
        self.master = np.zeros(self.shape, np.uint8)
        # Precomputed Nav2 inflation law, indexed by distance in cells.
        max_cells = int(np.ceil(params.inflation_radius / res)) + 2
        d = np.arange(max_cells + 1) * res
        law = np.where(d <= params.inscribed_radius, INSCRIBED,
                       (INSCRIBED - 1) * np.exp(-params.cost_scaling_factor * (d - params.inscribed_radius)))
        law[d > params.inflation_radius] = 0
        law[0] = LETHAL
        self._law = law.astype(np.uint8)

    def world_to_cell(self, x, y):
        res = self.p.resolution
        return ((np.asarray(y) - self.origin[1]) / res).astype(int), ((np.asarray(x) - self.origin[0]) / res).astype(int)

    def update_obstacles(self, pose, angles, ranges):
        """Nav2 obstacle layer: raytrace clearing then marking, from the believed pose."""
        res = self.p.resolution
        ny, nx = self.shape
        a = pose[2] + angles
        finite = np.isfinite(ranges)
        clear_to = np.where(finite, ranges - res, 0.0)   # inf returns do not clear (inf_is_valid: false)
        clear_to = np.minimum(clear_to, self.p.raytrace_max_range)
        steps = np.arange(0.0, self.p.raytrace_max_range, res * 0.8)
        mask = steps[None, :] < clear_to[:, None]
        cx = pose[0] + steps[None, :] * np.cos(a)[:, None]
        cy = pose[1] + steps[None, :] * np.sin(a)[:, None]
        j, i = self.world_to_cell(cx[mask], cy[mask])
        ok = (i >= 0) & (i < nx) & (j >= 0) & (j < ny)
        self.obstacle[j[ok], i[ok]] = False
        mark = finite & (ranges < self.p.obstacle_max_range)
        mx = pose[0] + ranges[mark] * np.cos(a[mark])
        my = pose[1] + ranges[mark] * np.sin(a[mark])
        j, i = self.world_to_cell(mx, my)
        ok = (i >= 0) & (i < nx) & (j >= 0) & (j < ny)
        self.obstacle[j[ok], i[ok]] = True

    def add_detections(self, cam_pose, dets, stamp):
        """Camera-frame detections -> global frame via the believed camera pose at capture."""
        if self.model is None:
            return
        for cls, z, bearing, rel_yaw, sx, sy in dets:
            x = cam_pose[0] + z * np.cos(cam_pose[2] + bearing)
            y = cam_pose[1] + z * np.sin(cam_pose[2] + bearing)
            self.model.add(CLASS_IDS[cls], x, y, cam_pose[2] + rel_yaw, sx, sy, stamp)

    def clear(self):
        """ClearEntireCostmap: drop sensor marks, keep the static map."""
        self.obstacle[:] = False
        if self.model is not None:
            self.model.clear()

    def update(self, now):
        lethal = self.static | self.obstacle
        dist_cells = ndimage.distance_transform_edt(~lethal)
        idx = np.minimum(np.rint(dist_cells).astype(int), len(self._law) - 1)
        self.master = np.ascontiguousarray(self._law[idx])
        if self.model is not None:
            self.model.prune(now)
            self.model.stamp(self.master, self.origin, self.p.resolution)
        return self.master

    def cost_at(self, x, y):
        j, i = self.world_to_cell(x, y)
        ny, nx = self.shape
        ok = (i >= 0) & (i < nx) & (j >= 0) & (j < ny)
        out = np.full(np.shape(i), LETHAL, np.uint8)
        out[ok] = self.master[j[ok], i[ok]]
        return out
