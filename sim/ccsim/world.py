"""Test worlds. The same definitions generate the Gazebo SDF worlds
(src/cc_gazebo/scripts/generate_worlds.py), so both simulators share layouts.

Heights matter: the LiDAR scans a plane at LIDAR_HEIGHT, so anything lower
(pallets, low boxes) is invisible to it, carts only show their thin legs,
and glass returns only near normal incidence.
"""
from dataclasses import dataclass, field

import numpy as np
from scipy import ndimage

from .geometry import box_segments, point_segment_distance

LIDAR_HEIGHT = 0.18
ROBOT_RADIUS = 0.25

WORLD_NAMES = ["empty_corridor", "cluttered_room", "glass_wall", "walking_actors", "kidnapped"]


@dataclass
class Obstacle:
    kind: str            # pallet | low_box | cart | shelf
    cls: str             # detector class: low_obstacle | cart | unknown
    x: float
    y: float
    yaw: float
    sx: float
    sy: float
    height: float

    @property
    def box(self):
        return np.array([self.x, self.y, self.yaw, self.sx, self.sy])

    def lidar_segments(self):
        if self.height > LIDAR_HEIGHT and self.kind != "cart":
            return box_segments(self.x, self.y, self.yaw, self.sx, self.sy)
        return np.zeros((0, 4))

    def lidar_circles(self):
        if self.kind != "cart":
            return np.zeros((0, 3))
        c, s = np.cos(self.yaw), np.sin(self.yaw)
        hx, hy = 0.5 * self.sx - 0.04, 0.5 * self.sy - 0.04
        legs = [(hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy)]
        return np.array([[self.x + c * a - s * b, self.y + s * a + c * b, 0.02] for a, b in legs])


@dataclass
class Actor:
    """Pedestrian walking back and forth along a lane. Like a Gazebo actor it
    never sidesteps; unlike one, it pauses while the robot blocks its way
    (someone stopping in front of a robot), so a collision means the robot
    drove into the person."""
    waypoints: np.ndarray
    speed: float
    phase: float           # seconds of offset into the cycle
    radius: float = 0.25   # collision/shoulder radius
    lidar_radius: float = 0.18
    yield_distance: float = 0.5   # surface gap ahead at which the person stops

    def __post_init__(self):
        self.waypoints = np.asarray(self.waypoints, float)
        seg = np.diff(self.waypoints, axis=0)
        self._lengths = np.hypot(seg[:, 0], seg[:, 1])
        self._cum = np.concatenate([[0.0], np.cumsum(self._lengths)])
        self.path_length = float(self._cum[-1])
        self.s = (self.phase * self.speed) % (2.0 * self.path_length)
        self.paused = False

    def state(self, t=None):
        """(x, y, vx, vy) at the current progress (t kept for API symmetry)."""
        s, direction = self.s, 1.0
        if s > self.path_length:
            s = 2.0 * self.path_length - s
            direction = -1.0
        k = int(np.clip(np.searchsorted(self._cum, s, side="right") - 1, 0, len(self._lengths) - 1))
        u = (s - self._cum[k]) / max(self._lengths[k], 1e-9)
        p0, p1 = self.waypoints[k], self.waypoints[k + 1]
        pos = p0 + u * (p1 - p0)
        speed = 0.0 if self.paused else self.speed
        vel = direction * speed * (p1 - p0) / max(self._lengths[k], 1e-9)
        return pos[0], pos[1], vel[0], vel[1]

    def advance(self, dt, robot_xy, robot_radius=ROBOT_RADIUS):
        x, y, _, _ = self.state()
        s, direction = self.s, 1.0
        if s > self.path_length:
            direction = -1.0
        k = int(np.clip(np.searchsorted(self._cum, min(s, 2 * self.path_length - s), side="right") - 1,
                        0, len(self._lengths) - 1))
        d = direction * (self.waypoints[k + 1] - self.waypoints[k]) / max(self._lengths[k], 1e-9)
        rx, ry = robot_xy[0] - x, robot_xy[1] - y
        ahead = rx * d[0] + ry * d[1]
        lateral = abs(-rx * d[1] + ry * d[0])
        gap = self.radius + robot_radius
        self.paused = 0.0 < ahead < gap + self.yield_distance and lateral < gap
        if not self.paused:
            self.s = (self.s + self.speed * dt) % (2.0 * self.path_length)


@dataclass
class World:
    name: str
    bounds: tuple
    walls: np.ndarray
    glass: np.ndarray = field(default_factory=lambda: np.zeros((0, 4)))
    obstacles: list = field(default_factory=list)
    actors: list = field(default_factory=list)
    start: np.ndarray = field(default_factory=lambda: np.zeros(3))
    goal: np.ndarray = field(default_factory=lambda: np.zeros(2))
    kidnap: tuple = None   # (time_s, true_pose)
    timeout: float = 120.0

    # --- geometry exposed to sensors -------------------------------------
    def lidar_opaque_segments(self):
        segs = [self.walls] + [o.lidar_segments() for o in self.obstacles]
        return np.vstack([s for s in segs if len(s)]) if segs else np.zeros((0, 4))

    def lidar_static_circles(self):
        c = [o.lidar_circles() for o in self.obstacles]
        c = [x for x in c if len(x)]
        return np.vstack(c) if c else np.zeros((0, 3))

    def camera_blocking_segments(self):
        """Walls and tall obstacles block the camera; glass does not."""
        segs = [self.walls] + [box_segments(*o.box) for o in self.obstacles if o.height > 0.5]
        return np.vstack(segs)

    def obstacle_boxes(self):
        return np.array([o.box for o in self.obstacles]) if self.obstacles else np.zeros((0, 5))

    def actor_states(self, t=None):
        return np.array([a.state() for a in self.actors]) if self.actors else np.zeros((0, 4))

    def step_actors(self, dt, robot_xy):
        for a in self.actors:
            a.advance(dt, robot_xy)

    def free_space_grid(self, res=0.05, clearance=ROBOT_RADIUS + 0.05, include_obstacles=True):
        """Boolean grid of cells where the robot centre fits (ground truth), plus origin."""
        x0, y0, x1, y1 = self.bounds
        nx, ny = int(np.ceil((x1 - x0) / res)), int(np.ceil((y1 - y0) / res))
        occ = np.zeros((ny, nx), bool)
        segs = [self.walls, self.glass]
        if include_obstacles:
            segs += [box_segments(*o.box) for o in self.obstacles]
        _rasterize_segments(occ, np.vstack([s for s in segs if len(s)]), x0, y0, res)
        if include_obstacles:
            for o in self.obstacles:
                _fill_box(occ, o, x0, y0, res)
        dist = ndimage.distance_transform_edt(~occ) * res
        return dist > clearance, (x0, y0)


def _rasterize_segments(grid, segs, x0, y0, res):
    ny, nx = grid.shape
    for x1, y1, x2, y2 in segs:
        n = int(np.hypot(x2 - x1, y2 - y1) / (0.5 * res)) + 2
        xs = np.linspace(x1, x2, n)
        ys = np.linspace(y1, y2, n)
        i = np.clip(((xs - x0) / res).astype(int), 0, nx - 1)
        j = np.clip(((ys - y0) / res).astype(int), 0, ny - 1)
        grid[j, i] = True


def _fill_box(grid, o, x0, y0, res):
    ny, nx = grid.shape
    r = 0.5 * np.hypot(o.sx, o.sy)
    i0, i1 = max(int((o.x - r - x0) / res), 0), min(int((o.x + r - x0) / res) + 1, nx)
    j0, j1 = max(int((o.y - r - y0) / res), 0), min(int((o.y + r - y0) / res) + 1, ny)
    if i0 >= i1 or j0 >= j1:
        return
    xs = x0 + (np.arange(i0, i1) + 0.5) * res
    ys = y0 + (np.arange(j0, j1) + 0.5) * res
    X, Y = np.meshgrid(xs, ys)
    c, s = np.cos(o.yaw), np.sin(o.yaw)
    lx = np.abs(c * (X - o.x) + s * (Y - o.y)) <= 0.5 * o.sx
    ly = np.abs(-s * (X - o.x) + c * (Y - o.y)) <= 0.5 * o.sy
    grid[j0:j1, i0:i1] |= lx & ly


def _rect_walls(x0, y0, x1, y1):
    return np.array([[x0, y0, x1, y0], [x1, y0, x1, y1], [x1, y1, x0, y1], [x0, y1, x0, y0]], float)


def reachable(world, res=0.05):
    free, (x0, y0) = world.free_space_grid(res)
    labels, _ = ndimage.label(free)
    s = labels[int((world.start[1] - y0) / res), int((world.start[0] - x0) / res)]
    g = labels[int((world.goal[1] - y0) / res), int((world.goal[0] - x0) / res)]
    return s != 0 and s == g


# --------------------------------------------------------------------------
# World builders. `rng` makes per-seed randomisation reproducible; the same
# seed gives the same world for every ablation config (paired design).
# --------------------------------------------------------------------------

def empty_corridor(rng):
    w = World("empty_corridor", (-0.5, -0.5, 30.5, 3.5), _rect_walls(0, 0, 30, 3))
    w.start = np.array([1.5, rng.uniform(1.2, 1.8), rng.normal(0, 0.1)])
    w.goal = np.array([28.5, rng.uniform(1.2, 1.8)])
    w.timeout = 120.0
    return w


OBSTACLE_KINDS = {
    #          cls,           sx,   sy,   height
    "pallet": ("low_obstacle", 1.2, 0.8, 0.14),
    "low_box": ("low_obstacle", 0.45, 0.45, 0.12),
    "cart": ("cart", 0.9, 0.55, 0.95),
    "shelf": ("unknown", 0.6, 0.6, 1.2),
}


def cluttered_room(rng):
    for _ in range(200):
        w = World("cluttered_room", (-0.5, -0.5, 12.5, 10.5), _rect_walls(0, 0, 12, 10))
        w.start = np.array([1.0, 1.0, np.arctan2(8.0, 10.0)])
        w.goal = np.array([11.0, 9.0])
        w.timeout = 120.0
        kinds = ["pallet"] * 3 + ["low_box"] * 3 + ["cart"] * 2 + ["shelf"]
        axis = (w.goal - w.start[:2]) / np.linalg.norm(w.goal - w.start[:2])
        normal = np.array([-axis[1], axis[0]])
        placed = []
        for kind in kinds:
            cls, sx, sy, h = OBSTACLE_KINDS[kind]
            for _ in range(100):
                t = rng.uniform(0.15, 0.85)
                p = w.start[:2] + t * (w.goal - w.start[:2]) + rng.normal(0, 1.2) * normal
                r = 0.5 * np.hypot(sx, sy)
                if not (r + 0.2 < p[0] < 12 - r - 0.2 and r + 0.2 < p[1] < 10 - r - 0.2):
                    continue
                if np.linalg.norm(p - w.start[:2]) < 1.4 or np.linalg.norm(p - w.goal) < 1.4:
                    continue
                if any(np.hypot(*(p - q[:2])) < r + q[2] + 0.75 for q in placed):
                    continue
                placed.append((p[0], p[1], r))
                w.obstacles.append(Obstacle(kind, cls, p[0], p[1], rng.uniform(-np.pi, np.pi), sx, sy, h))
                break
        if len(w.obstacles) == len(kinds) and reachable(w):
            return w
    raise RuntimeError("could not generate a feasible cluttered room")


def glass_wall(rng):
    w = World("glass_wall", (-0.5, -0.5, 20.5, 8.5), _rect_walls(0, 0, 20, 8))
    tilt = np.deg2rad(rng.uniform(0.0, 25.0)) * rng.choice([-1.0, 1.0])
    # Glass partition across the lobby with a 2 m doorway at the top.
    w.glass = np.array([[10.0 - 3.0 * np.tan(tilt), 0.0, 10.0 + 3.0 * np.tan(tilt), 6.0]])
    w.start = np.array([1.5, rng.uniform(1.0, 4.5), 0.0])
    w.goal = np.array([18.5, rng.uniform(1.0, 4.5)])
    w.timeout = 120.0
    return w


def walking_actors(rng, n_actors=5):
    w = World("walking_actors", (-0.5, -0.5, 30.5, 4.5), _rect_walls(0, 0, 30, 4))
    w.start = np.array([1.5, 2.0, 0.0])
    w.goal = np.array([28.5, 2.0])
    w.timeout = 150.0
    for _ in range(n_actors):
        lane = rng.uniform(0.7, 3.3)
        xa, xb = rng.uniform(3.0, 12.0), rng.uniform(18.0, 27.0)
        w.actors.append(Actor([[xa, lane], [xb, lane]], rng.uniform(0.8, 1.3), rng.uniform(0, 60.0)))
    return w


def kidnapped(rng):
    walls = [_rect_walls(0, 0, 16, 12), _rect_walls(5, 4, 11, 8)]
    # Asymmetric features so the map is not self-similar.
    walls.append(_rect_walls(2.2, 8.7, 2.8, 9.3))                 # pillar
    walls.append(np.array([[13.0, 0.0, 13.0, 2.0], [8.0, 12.0, 8.0, 10.5], [0.0, 6.0, 1.5, 6.0]]))
    w = World("kidnapped", (-0.5, -0.5, 16.5, 12.5), np.vstack(walls))
    w.start = np.array([2.0, 2.0, 0.0])
    w.goal = np.array([14.0, 10.0])
    w.timeout = 180.0
    candidates = [(14.5, 2.0), (2.0, 10.5), (11.0, 10.0), (3.0, 5.0), (13.5, 6.0)]
    x, y = candidates[rng.integers(len(candidates))]
    w.kidnap = (10.0, np.array([x, y, rng.uniform(-np.pi, np.pi)]))
    return w


def drift_loop(rng):
    """Open floor for the 20 m EKF drift loop (milestone 3)."""
    w = World("drift_loop", (-2.0, -2.0, 8.0, 8.0), _rect_walls(-1.5, -1.5, 7.5, 7.5))
    w.start = np.zeros(3)
    return w


BUILDERS = {
    "empty_corridor": empty_corridor,
    "cluttered_room": cluttered_room,
    "glass_wall": glass_wall,
    "walking_actors": walking_actors,
    "kidnapped": kidnapped,
    "drift_loop": drift_loop,
}


def make_world(name, seed):
    return BUILDERS[name](np.random.default_rng([seed, 7]))


def min_wall_distance(world, x, y):
    segs = np.vstack([world.walls, world.glass]) if len(world.glass) else world.walls
    return float(point_segment_distance(x, y, segs).min())
