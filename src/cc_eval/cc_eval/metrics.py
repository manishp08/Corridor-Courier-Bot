"""ROS-free metric and statistics helpers shared by the Gazebo harness and
the 2D surrogate simulator, so both produce identical tables."""
import math

import numpy as np
from scipy import stats


def wilson_ci(successes, n, z=1.96):
    """Wilson score interval for a binomial proportion. Returns (p, lo, hi)."""
    if n == 0:
        return float("nan"), float("nan"), float("nan")
    p = successes / n
    denom = 1.0 + z * z / n
    centre = (p + z * z / (2 * n)) / denom
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / denom
    return p, max(0.0, centre - half), min(1.0, centre + half)


def bootstrap_ci(x, stat=np.mean, n_boot=5000, alpha=0.05, seed=0):
    """Percentile bootstrap CI of stat(x), NaNs dropped. Returns (value, lo, hi)."""
    x = np.asarray(x, float)
    x = x[~np.isnan(x)]
    if len(x) == 0:
        return float("nan"), float("nan"), float("nan")
    rng = np.random.default_rng(seed)
    boots = stat(x[rng.integers(0, len(x), (n_boot, len(x)))], axis=1)
    return float(stat(x)), float(np.quantile(boots, alpha / 2)), float(np.quantile(boots, 1 - alpha / 2))


def mcnemar_exact(a, b):
    """Exact McNemar test for paired binary outcomes (same seeds). Two-sided p."""
    a, b = np.asarray(a, bool), np.asarray(b, bool)
    n01 = int(np.sum(~a & b))
    n10 = int(np.sum(a & ~b))
    n = n01 + n10
    if n == 0:
        return 1.0
    return float(min(1.0, 2 * stats.binom.cdf(min(n01, n10), n, 0.5)))


def wilcoxon_paired(a, b):
    """Wilcoxon signed-rank test on paired samples; p = 1 if all differences are 0."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    ok = ~(np.isnan(a) | np.isnan(b))
    d = a[ok] - b[ok]
    if len(d) == 0 or np.all(d == 0):
        return 1.0
    return float(stats.wilcoxon(a[ok], b[ok], zero_method="zsplit").pvalue)


def ate_rmse(est_xy, gt_xy):
    """Absolute trajectory error (RMSE of position), both in the map frame.
    Equivalent to `evo_ape` translation RMSE without alignment."""
    est_xy, gt_xy = np.asarray(est_xy, float), np.asarray(gt_xy, float)
    return float(np.sqrt(np.mean(np.sum((est_xy - gt_xy) ** 2, axis=1))))


def path_length(xy):
    xy = np.asarray(xy, float)
    if len(xy) < 2:
        return 0.0
    return float(np.sum(np.hypot(*np.diff(xy, axis=0).T)))


def min_clearance(robot_xy, people_xy, robot_radius, person_radius):
    """Minimum surface-to-surface distance to any person over a run.
    robot_xy: (T, 2); people_xy: (T, P, 2). Negative = overlap."""
    robot_xy, people_xy = np.asarray(robot_xy, float), np.asarray(people_xy, float)
    if people_xy.size == 0:
        return float("nan")
    d = np.linalg.norm(people_xy - robot_xy[:, None, :], axis=2)
    return float(d.min() - robot_radius - person_radius)


def count_contacts(contact_sets):
    """Number of collision events = rising edges of per-object contact."""
    prev, n = set(), 0
    for cur in contact_sets:
        n += len(set(cur) - prev)
        prev = set(cur)
    return n


def relative_reduction(before, after):
    """(before - after) / before, in percent."""
    if before == 0 or np.isnan(before):
        return float("nan")
    return 100.0 * (before - after) / before
