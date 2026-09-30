"""Global planner (A* on the costmap, NavFn-like cost term) and two local
controllers: a DWB-style trajectory sampler and an MPPI sampler."""
import heapq
from dataclasses import dataclass

import numpy as np

from .costmap import INSCRIBED
from .geometry import wrap

SQRT2 = np.sqrt(2.0)


@dataclass
class PlannerParams:
    resolution: float = 0.1       # planning grid (max-pooled costmap)
    cost_factor: float = 3.0      # extra cost per unit of normalised cell cost
    goal_tolerance: float = 0.3   # search this far for a free goal cell


def plan(costmap, start, goal, params=PlannerParams()):
    """A* over an 8-connected grid. Returns an (N, 2) path in metres or None."""
    k = int(round(params.resolution / costmap.p.resolution))
    m = costmap.master
    ny, nx = m.shape[0] // k, m.shape[1] // k
    grid = m[:ny * k, :nx * k].reshape(ny, k, nx, k).max(axis=(1, 3))
    res = params.resolution
    ox, oy = costmap.origin

    def cell(p):
        return int((p[1] - oy) / res), int((p[0] - ox) / res)

    sj, si = cell(start)
    gj, gi = cell(goal)
    if not (0 <= sj < ny and 0 <= si < nx and 0 <= gj < ny and 0 <= gi < nx):
        return None
    blocked = grid >= INSCRIBED
    if blocked[gj, gi]:
        r = int(np.ceil(params.goal_tolerance / res))
        js, is_ = np.mgrid[max(gj - r, 0):min(gj + r + 1, ny), max(gi - r, 0):min(gi + r + 1, nx)]
        free = ~blocked[js, is_]
        if not free.any():
            return None
        d = np.hypot(js - gj, is_ - gi) + np.where(free, 0, 1e9)
        a = np.unravel_index(np.argmin(d), d.shape)
        gj, gi = int(js[a]), int(is_[a])
    # The robot may start inside the inflated zone (e.g. after a bump):
    # let it leave through cells within 0.4 m of the start.
    esc = int(np.ceil(0.4 / res))
    blocked[max(sj - esc, 0):sj + esc + 1, max(si - esc, 0):si + esc + 1] &= grid[max(sj - esc, 0):sj + esc + 1, max(si - esc, 0):si + esc + 1] >= 254
    blocked[sj, si] = False
    cost = (1.0 + params.cost_factor * grid.astype(float) / 252.0).ravel().tolist()
    blk = blocked.ravel().tolist()
    start_i, goal_i = sj * nx + si, gj * nx + gi
    g = {start_i: 0.0}
    parent = {start_i: -1}
    closed = set()
    heap = [(0.0, start_i)]
    nbrs = [(-1, -1, SQRT2), (-1, 0, 1.0), (-1, 1, SQRT2), (0, -1, 1.0),
            (0, 1, 1.0), (1, -1, SQRT2), (1, 0, 1.0), (1, 1, SQRT2)]
    while heap:
        _, cur = heapq.heappop(heap)
        if cur in closed:
            continue
        if cur == goal_i:
            break
        closed.add(cur)
        cj, ci = divmod(cur, nx)
        gc = g[cur]
        for dj, di, step in nbrs:
            nj, ni = cj + dj, ci + di
            if nj < 0 or nj >= ny or ni < 0 or ni >= nx:
                continue
            n = nj * nx + ni
            if blk[n] or n in closed:
                continue
            ng = gc + step * cost[n]
            if ng < g.get(n, 1e18):
                g[n] = ng
                parent[n] = cur
                h = max(abs(nj - gj), abs(ni - gi)) + (SQRT2 - 1) * min(abs(nj - gj), abs(ni - gi))
                heapq.heappush(heap, (ng + h, n))
    if goal_i not in parent:
        return None
    out = []
    n = goal_i
    while n != -1:
        j, i = divmod(n, nx)
        out.append((ox + (i + 0.5) * res, oy + (j + 0.5) * res))
        n = parent[n]
    path = np.array(out[::-1])
    path[0] = start[:2]
    path[-1] = goal[:2]
    return path


def path_blocked(costmap, path, pose, lookahead=2.0):
    if path is None:
        return True
    i0 = int(np.argmin(np.hypot(path[:, 0] - pose[0], path[:, 1] - pose[1])))
    seg = path[i0:]
    d = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(seg, axis=0).T))])
    seg = seg[(d <= lookahead) & (d >= 0.4)]
    if len(seg) == 0:
        return False
    return bool(np.any(costmap.cost_at(seg[:, 0], seg[:, 1]) >= INSCRIBED))


def local_goal(path, pose, lookahead):
    d0 = np.hypot(path[:, 0] - pose[0], path[:, 1] - pose[1])
    i0 = int(np.argmin(d0))
    seg = path[i0:]
    d = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(seg, axis=0).T))]) if len(seg) > 1 else np.zeros(1)
    k = int(np.searchsorted(d, lookahead))
    return seg[min(k, len(seg) - 1)], seg[:max(k + 1, 2)]


@dataclass
class RobotLimits:
    v_max: float = 0.5
    w_max: float = 1.0
    acc_v: float = 1.0
    acc_w: float = 2.0
    dt: float = 0.1


def _rollout(pose, v, w, steps, dt):
    """v, w: (S,) constant or (S, T) sequences. Returns xs, ys, ths of shape (S, T)."""
    S = v.shape[0]
    if v.ndim == 1:
        v = np.repeat(v[:, None], steps, axis=1)
        w = np.repeat(w[:, None], steps, axis=1)
    th = pose[2] + np.cumsum(w * dt, axis=1)
    th_mid = th - 0.5 * w * dt
    xs = pose[0] + np.cumsum(v * np.cos(th_mid) * dt, axis=1)
    ys = pose[1] + np.cumsum(v * np.sin(th_mid) * dt, axis=1)
    return xs, ys, th


def _path_distance(xs, ys, pts):
    d = np.hypot(xs[..., None] - pts[:, 0], ys[..., None] - pts[:, 1])
    return d.min(axis=-1)


def _admissible(c, start_cost):
    """Trajectories may not enter inscribed/lethal cells. If the robot already
    sits in one (a person stepped close, or after a bump) it may only move
    to cells that are no worse, so it can back out instead of freezing."""
    if start_cost < INSCRIBED:
        return ~np.any(c >= INSCRIBED, axis=1)
    return ~np.any(c > start_cost, axis=1) & (c[:, -1] <= start_cost)


class DwbController:
    """Samples constant (v, w) inside the dynamic window, simulates 1.5 s,
    rejects trajectories that touch inscribed/lethal cells and scores the rest
    with path-distance, goal-distance, heading and obstacle-cost critics."""

    name = "dwb"

    def __init__(self, limits=RobotLimits(), sim_time=1.5, v_samples=8, w_samples=15,
                 w_path=1.0, w_goal=1.5, w_heading=0.3, w_obstacle=1.5, w_speed=0.3, lookahead=1.5):
        self.l = limits
        self.T = int(round(sim_time / limits.dt))
        self.vs, self.ws = v_samples, w_samples
        self.w_path, self.w_goal, self.w_heading = w_path, w_goal, w_heading
        self.w_obs, self.w_speed, self.lookahead = w_obstacle, w_speed, lookahead

    def reset(self):
        pass

    def compute(self, pose, v0, w0, path, costmap):
        l = self.l
        goal_pt, window = local_goal(path, pose, self.lookahead)
        heading_err = wrap(np.arctan2(goal_pt[1] - pose[1], goal_pt[0] - pose[0]) - pose[2])
        vmin = max(0.0, v0 - l.acc_v * l.dt * 3)
        vmax = min(l.v_max, v0 + l.acc_v * l.dt * 3)
        if abs(heading_err) > 1.0:  # rotate towards the path first
            vmax = min(vmax, 0.05)
            vmin = 0.0
        wlo = max(-l.w_max, w0 - l.acc_w * l.dt * 3)
        whi = min(l.w_max, w0 + l.acc_w * l.dt * 3)
        V, W = np.meshgrid(np.linspace(vmin, vmax, self.vs), np.linspace(wlo, whi, self.ws))
        V, W = V.ravel(), W.ravel()
        xs, ys, th = _rollout(pose, V, W, self.T, l.dt)
        c = costmap.cost_at(xs, ys)
        ok = _admissible(c, costmap.cost_at(np.array([pose[0]]), np.array([pose[1]]))[0])
        if not ok.any():
            return None
        obs = c.max(axis=1) / 252.0
        path_d = _path_distance(xs[:, -1], ys[:, -1], window)
        goal_d = np.hypot(goal_pt[0] - xs[:, -1], goal_pt[1] - ys[:, -1])
        head = np.abs(wrap(np.arctan2(goal_pt[1] - ys[:, -1], goal_pt[0] - xs[:, -1]) - th[:, -1]))
        score = (self.w_path * path_d + self.w_goal * goal_d + self.w_heading * head
                 + self.w_obs * obs - self.w_speed * V)
        score[~ok] = np.inf
        b = int(np.argmin(score))
        return float(V[b]), float(W[b])


class MppiController:
    """Model Predictive Path Integral control over a unicycle model."""

    name = "mppi"

    def __init__(self, limits=RobotLimits(), rng=None, samples=400, horizon=20, sigma_v=0.2, sigma_w=0.5,
                 temperature=0.05, w_obstacle=2.0, w_path=1.5, w_goal=2.0, w_heading=0.3, w_speed=1.5,
                 lookahead=2.0, collision_cost=1000.0):
        self.l = limits
        self.rng = rng or np.random.default_rng(0)
        self.K, self.T = samples, horizon
        self.sig = np.array([sigma_v, sigma_w])
        self.lam = temperature
        self.w_obs, self.w_path, self.w_goal, self.w_heading = w_obstacle, w_path, w_goal, w_heading
        self.w_speed = w_speed
        self.lookahead = lookahead
        self.collision_cost = collision_cost
        self.reset()

    def reset(self):
        self.U = np.zeros((self.T, 2))

    def compute(self, pose, v0, w0, path, costmap):
        l = self.l
        goal_pt, window = local_goal(path, pose, self.lookahead)
        self.U = np.vstack([self.U[1:], self.U[-1:]])
        eps = self.rng.normal(0.0, 1.0, (self.K, self.T, 2)) * self.sig
        eps[0] = 0.0                                  # keep the nominal sequence as a sample
        U = self.U[None] + eps
        U[..., 0] = np.clip(U[..., 0], 0.0, l.v_max)
        U[..., 1] = np.clip(U[..., 1], -l.w_max, l.w_max)
        xs, ys, th = _rollout(pose, U[..., 0], U[..., 1], self.T, l.dt)
        c = costmap.cost_at(xs, ys)
        collide = ~_admissible(c, costmap.cost_at(np.array([pose[0]]), np.array([pose[1]]))[0])
        cost = self.w_obs * (c / 252.0).mean(axis=1)
        cost += self.w_path * _path_distance(xs[:, ::4], ys[:, ::4], window).mean(axis=1)
        cost += self.w_goal * np.hypot(goal_pt[0] - xs[:, -1], goal_pt[1] - ys[:, -1])
        cost += self.w_heading * np.abs(wrap(np.arctan2(goal_pt[1] - ys[:, -1], goal_pt[0] - xs[:, -1]) - th[:, -1]))
        cost += self.w_speed * (1.0 - U[..., 0].mean(axis=1) / l.v_max)   # prefer-forward critic
        cost += self.collision_cost * collide
        if collide.all():
            self.reset()
            return None
        wts = np.exp(-(cost - cost.min()) / self.lam)
        wts /= wts.sum()
        self.U = np.einsum("k,ktc->tc", wts, U)
        v, w = self.U[0]
        return float(np.clip(v, 0.0, l.v_max)), float(np.clip(w, -l.w_max, l.w_max))
