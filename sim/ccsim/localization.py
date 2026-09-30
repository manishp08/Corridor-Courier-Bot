"""Odometry integration, EKF (robot_localization-style twist fusion) and AMCL."""
from dataclasses import dataclass

import numpy as np
from scipy import ndimage

from .geometry import compose, inverse, wrap


class WheelOdometry:
    """Config A: odom -> base_link straight from integrated wheel twist."""

    def __init__(self):
        self.pose = np.zeros(3)

    def update(self, v, w, dt, gyro=None):
        th = self.pose[2] + 0.5 * w * dt
        self.pose = np.array([self.pose[0] + v * np.cos(th) * dt,
                              self.pose[1] + v * np.sin(th) * dt, wrap(self.pose[2] + w * dt)])
        return self.pose


@dataclass(frozen=True)
class EkfParams:
    """Measurement variances. 'tuned' is what cc_localization/config/ekf.yaml encodes."""
    r_wheel_v: float = 0.02 ** 2
    r_wheel_w: float = 0.08 ** 2    # distrust encoder yaw rate: it slips on polished floors
    r_gyro_w: float = 0.005 ** 2
    q_v: float = 1.0 ** 2           # process noise ~ accel limits (per s)
    q_w: float = 2.0 ** 2


EKF_TUNED = EkfParams()
# What you get by leaving the diff-drive plugin's tiny twist covariance and a
# generic IMU covariance in place: the filter trusts the slipping wheels.
EKF_UNTUNED = EkfParams(r_wheel_v=1e-6, r_wheel_w=1e-6, r_gyro_w=0.02 ** 2)


class Ekf:
    """5-state [x, y, yaw, v, w] EKF, fusing wheel v, wheel w and gyro w (two_d_mode)."""

    def __init__(self, params=EKF_TUNED):
        self.p = params
        self.x = np.zeros(5)
        self.P = np.diag([1e-6, 1e-6, 1e-6, 1e-2, 1e-2])

    def predict(self, dt):
        x, y, th, v, w = self.x
        self.x = np.array([x + v * np.cos(th) * dt, y + v * np.sin(th) * dt, wrap(th + w * dt), v, w])
        F = np.eye(5)
        F[0, 2], F[0, 3] = -v * np.sin(th) * dt, np.cos(th) * dt
        F[1, 2], F[1, 3] = v * np.cos(th) * dt, np.sin(th) * dt
        F[2, 4] = dt
        Q = np.diag([1e-6 * dt, 1e-6 * dt, 1e-6 * dt, self.p.q_v * dt, self.p.q_w * dt])
        self.P = F @ self.P @ F.T + Q

    def _update_scalar(self, idx, z, r):
        S = self.P[idx, idx] + r
        K = self.P[:, idx] / S
        self.x = self.x + K * (z - self.x[idx])
        self.x[2] = wrap(self.x[2])
        self.P = self.P - np.outer(K, self.P[idx, :])

    def update(self, v, w, dt, gyro=None):
        self.predict(dt)
        self._update_scalar(3, v, self.p.r_wheel_v)
        self._update_scalar(4, w, self.p.r_wheel_w)
        if gyro is not None:
            self._update_scalar(4, gyro, self.p.r_gyro_w)
        return self.x[:3].copy()

    @property
    def pose(self):
        return self.x[:3].copy()


@dataclass
class AmclParams:
    """Mirrors cc_localization/config/amcl.yaml."""
    n_particles: int = 500
    alphas: tuple = (0.2, 0.2, 0.2, 0.2)
    sigma_hit: float = 0.2
    z_hit: float = 0.95
    z_rand: float = 0.05
    max_beams: int = 60
    laser_max_range: float = 8.0
    update_min_d: float = 0.25
    update_min_a: float = 0.2
    # Nav2 defaults (0 = off). Random-particle injection in a long corridor
    # lets aliased particles hijack the estimate; kidnaps are handled by
    # LocalizationMonitor + global re-initialisation instead.
    recovery_alpha_slow: float = 0.0
    recovery_alpha_fast: float = 0.0
    global_particles: int = 5000
    map_res: float = 0.05


class Amcl:
    """Augmented MCL with a likelihood-field sensor model on the static (LiDAR-built) map."""

    def __init__(self, world, init_pose, rng, params=None, init_sigma=(0.1, 0.1, 0.05)):
        self.p = params or AmclParams()
        self.rng = rng
        x0, y0, x1, y1 = world.bounds
        res = self.p.map_res
        self.origin = np.array([x0, y0])
        free, _ = world.free_space_grid(res, clearance=0.0, include_obstacles=False)
        occ = ~free
        occ[0, :] = occ[-1, :] = occ[:, 0] = occ[:, -1] = False
        self.field = ndimage.distance_transform_edt(~occ) * res
        free_cells, _ = world.free_space_grid(res, clearance=0.3, include_obstacles=False)
        self.free_xy = (np.argwhere(free_cells)[:, ::-1] + 0.5) * res + self.origin
        self.particles = np.column_stack([
            init_pose[0] + rng.normal(0, init_sigma[0], self.p.n_particles),
            init_pose[1] + rng.normal(0, init_sigma[1], self.p.n_particles),
            wrap(init_pose[2] + rng.normal(0, init_sigma[2], self.p.n_particles))])
        self.weights = np.full(self.p.n_particles, 1.0 / self.p.n_particles)
        self.last_odom = None
        self.map_to_odom = None
        self.estimate = np.array(init_pose, float)
        self.w_slow = self.w_fast = 0.0
        self.updates = 0
        self.forced_updates = 0
        self.global_inits = 0

    def _motion(self, d):
        a1, a2, a3, a4 = self.p.alphas
        trans = np.hypot(d[0], d[1])
        rot1 = np.arctan2(d[1], d[0]) if trans > 0.01 else 0.0
        rot2 = wrap(d[2] - rot1)
        # Nav2's diff model folds backward motion into a small rotation.
        rot1_n = min(abs(wrap(rot1)), abs(wrap(rot1 - np.pi)))
        rot2_n = min(abs(wrap(rot2)), abs(wrap(rot2 - np.pi)))
        n = len(self.particles)
        r1 = rot1 - self.rng.normal(0, np.sqrt(a1 * rot1_n ** 2 + a2 * trans ** 2), n)
        tr = trans - self.rng.normal(0, np.sqrt(a3 * trans ** 2 + a4 * (rot1_n ** 2 + rot2_n ** 2)), n)
        r2 = rot2 - self.rng.normal(0, np.sqrt(a1 * rot2_n ** 2 + a2 * trans ** 2), n)
        th = self.particles[:, 2]
        self.particles[:, 0] += tr * np.cos(th + r1)
        self.particles[:, 1] += tr * np.sin(th + r1)
        self.particles[:, 2] = wrap(th + r1 + r2)

    def _likelihood(self, angles, ranges):
        valid = np.isfinite(ranges)
        idx = np.flatnonzero(valid)
        if len(idx) == 0:
            return np.ones(len(self.particles))
        step = max(1, len(idx) // self.p.max_beams)
        idx = idx[::step]
        a, r = angles[idx], ranges[idx]
        P = self.particles
        ex = P[:, 0:1] + r[None, :] * np.cos(P[:, 2:3] + a[None, :])
        ey = P[:, 1:2] + r[None, :] * np.sin(P[:, 2:3] + a[None, :])
        res = self.p.map_res
        i = ((ex - self.origin[0]) / res).astype(int)
        j = ((ey - self.origin[1]) / res).astype(int)
        ny, nx = self.field.shape
        inside = (i >= 0) & (i < nx) & (j >= 0) & (j < ny)
        d = np.where(inside, self.field[np.clip(j, 0, ny - 1), np.clip(i, 0, nx - 1)], np.inf)
        pz = self.p.z_hit * np.exp(-0.5 * (d / self.p.sigma_hit) ** 2) + self.p.z_rand / self.p.laser_max_range
        return 1.0 + np.sum(pz ** 3, axis=1)

    def update(self, odom_pose, angles, ranges):
        """Returns True if a filter update ran. Keeps map->odom current either way."""
        if self.last_odom is None:
            self.last_odom = odom_pose.copy()
            self.map_to_odom = compose(self.estimate, inverse(odom_pose))
            return False
        d = compose(inverse(self.last_odom), odom_pose)
        moved = np.hypot(d[0], d[1]) >= self.p.update_min_d or abs(d[2]) >= self.p.update_min_a
        if not moved:
            if self.forced_updates <= 0:
                return False
            self.forced_updates -= 1
        self.last_odom = odom_pose.copy()
        self._motion(d)
        lik = self._likelihood(angles, ranges)
        w = self.weights * lik
        w_avg = float(np.mean(lik))
        if self.p.recovery_alpha_slow > 0.0:
            self._track_weights(w_avg)
        w /= w.sum()
        self.weights = w
        self._estimate()
        self._resample()
        self.map_to_odom = compose(self.estimate, inverse(odom_pose))
        self.updates += 1
        return True

    def _track_weights(self, w_avg):
        self.w_slow += self.p.recovery_alpha_slow * (w_avg - self.w_slow) if self.w_slow else w_avg
        self.w_fast += self.p.recovery_alpha_fast * (w_avg - self.w_fast) if self.w_fast else w_avg

    def _estimate(self):
        """Mean of the particle cluster with the largest total weight (as AMCL does),
        so a single lucky random particle cannot hijack the estimate."""
        P, w = self.particles, self.weights
        sub = np.argsort(w)[-100:]
        d = np.hypot(P[sub, 0:1] - P[None, :, 0], P[sub, 1:2] - P[None, :, 1])
        mass = (d < 0.5) @ w
        c = P[sub[int(np.argmax(mass))]]
        near = np.hypot(P[:, 0] - c[0], P[:, 1] - c[1]) < 1.0
        wn = w * near
        wn /= wn.sum()
        x, y = np.sum(wn * P[:, 0]), np.sum(wn * P[:, 1])
        th = np.arctan2(np.sum(wn * np.sin(P[:, 2])), np.sum(wn * np.cos(P[:, 2])))
        self.estimate = np.array([x, y, th])

    def _resample(self):
        n = len(self.particles)
        if n > self.p.n_particles and np.std(self.particles[:, 0]) + np.std(self.particles[:, 1]) < 1.0:
            n = self.p.n_particles     # KLD-style shrink once the cloud has converged
        p_inject = max(0.0, 1.0 - self.w_fast / self.w_slow) if self.w_slow > 0 else 0.0
        cum = np.cumsum(self.weights)
        u = (self.rng.random() + np.arange(n)) / n
        idx = np.minimum(np.searchsorted(cum, u), len(self.particles) - 1)
        new = self.particles[idx].copy()
        inject = self.rng.random(n) < p_inject
        k = int(inject.sum())
        if k:
            pts = self.free_xy[self.rng.integers(len(self.free_xy), size=k)]
            new[inject, 0:2] = pts
            new[inject, 2] = self.rng.uniform(-np.pi, np.pi, k)
        self.particles = new
        self.weights = np.full(n, 1.0 / n)

    def match_ratio(self, pose, angles, ranges, tol=0.2):
        """Fraction of valid beams whose endpoint lies within tol of a map obstacle."""
        valid = np.isfinite(ranges)
        if valid.sum() < 10:
            return 1.0
        a, r = angles[valid], ranges[valid]
        ex = pose[0] + r * np.cos(pose[2] + a)
        ey = pose[1] + r * np.sin(pose[2] + a)
        i = ((ex - self.origin[0]) / self.p.map_res).astype(int)
        j = ((ey - self.origin[1]) / self.p.map_res).astype(int)
        ny, nx = self.field.shape
        inside = (i >= 0) & (i < nx) & (j >= 0) & (j < ny)
        d = np.where(inside, self.field[np.clip(j, 0, ny - 1), np.clip(i, 0, nx - 1)], np.inf)
        return float(np.mean(d < tol))

    def global_init(self, forced_updates=10):
        """/reinitialize_global_localization: uniform particles over free space."""
        n = self.p.global_particles
        pts = self.free_xy[self.rng.integers(len(self.free_xy), size=n)]
        self.particles = np.column_stack([pts, self.rng.uniform(-np.pi, np.pi, n)])
        self.weights = np.full(n, 1.0 / n)
        self.forced_updates = forced_updates
        self.global_inits += 1

    def belief(self, odom_pose):
        return compose(self.map_to_odom, odom_pose)


class LocalizationMonitor:
    """Detects a lost/kidnapped robot from the scan-to-map match ratio and
    triggers global re-localisation (cc_localization/localization_monitor.py)."""

    def __init__(self, threshold=0.35, hold_time=3.0, cooldown=10.0):
        self.threshold, self.hold_time, self.cooldown = threshold, hold_time, cooldown
        self.bad_since = None
        self.last_trigger = -1e9
        self.triggers = 0

    def step(self, t, amcl, belief, angles, ranges):
        ratio = amcl.match_ratio(belief, angles, ranges)
        if ratio >= self.threshold:
            self.bad_since = None
            return False
        self.bad_since = t if self.bad_since is None else self.bad_since
        if t - self.bad_since >= self.hold_time and t - self.last_trigger >= self.cooldown:
            amcl.global_init()
            self.last_trigger = t
            self.bad_since = None
            self.triggers += 1
            return True
        return False
