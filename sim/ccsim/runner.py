"""One navigation episode: true physics + sensors + the configured stack."""
import time
from dataclasses import dataclass

import numpy as np

from .costmap import PERSON_AWARE, PERSON_PLAIN, STATIC_CLASSES, Costmap, CostmapParams
from .geometry import compose, point_box_distance, point_segment_distance, wrap
from .localization import EKF_TUNED, EKF_UNTUNED, Amcl, Ekf, LocalizationMonitor, WheelOdometry
from .planning import DwbController, MppiController, RobotLimits, path_blocked, plan
from .sensors import CameraDetector, Gyro, Lidar, WheelEncoders
from .world import ROBOT_RADIUS, make_world

DT = 0.1
PERSONAL_SPACE = 0.5   # m, surface to surface


@dataclass(frozen=True)
class StackConfig:
    name: str
    use_ekf: bool
    use_camera: bool
    people_aware: bool
    ekf_params: object = EKF_TUNED


CONFIGS = {
    "A": StackConfig("A", use_ekf=False, use_camera=False, people_aware=False),
    "B": StackConfig("B", use_ekf=True, use_camera=False, people_aware=False),
    "C": StackConfig("C", use_ekf=True, use_camera=True, people_aware=False),
    "D": StackConfig("D", use_ekf=True, use_camera=True, people_aware=True),
    # Not part of the ablation; used to show the untuned-covariance pitfall.
    "B_untuned": StackConfig("B_untuned", use_ekf=True, use_camera=False, people_aware=False, ekf_params=EKF_UNTUNED),
}


@dataclass
class NavParams:
    """Shared by every config and world. Never tuned per scenario."""
    goal_tolerance: float = 0.25
    success_radius: float = 0.5     # true distance for an episode to count as success
    replan_period: float = 1.0
    costmap_period: float = 0.2
    progress_radius: float = 0.5
    progress_time: float = 10.0
    max_recoveries: int = 6
    stuck_time: float = 2.0         # controller has no admissible trajectory this long -> recovery


class Robot:
    """Ground-truth diff-drive body with acceleration limits and hard contacts."""

    def __init__(self, pose, limits):
        self.pose = np.array(pose, float)
        self.v = self.w = 0.0
        self.l = limits

    def step(self, v_cmd, w_cmd, world, actors_now, rng):
        l = self.l
        v_cmd = np.clip(v_cmd, -l.v_max, l.v_max)
        w_cmd = np.clip(w_cmd, -l.w_max, l.w_max)
        self.v += np.clip(v_cmd - self.v, -l.acc_v * DT, l.acc_v * DT)
        self.w += np.clip(w_cmd - self.w, -l.acc_w * DT, l.acc_w * DT)
        v = self.v * (1.0 + rng.normal(0, 0.02))
        w = self.w * (1.0 + rng.normal(0, 0.02))
        th = self.pose[2] + 0.5 * w * DT
        new = np.array([self.pose[0] + v * np.cos(th) * DT, self.pose[1] + v * np.sin(th) * DT, wrap(self.pose[2] + w * DT)])
        if static_clearance(world, new[0], new[1]) < ROBOT_RADIUS:
            # Blocked: advance only up to the contact point (bisection); rotation still possible.
            lo, hi = 0.0, 1.0
            for _ in range(10):
                mid = 0.5 * (lo + hi)
                p = self.pose[:2] + mid * (new[:2] - self.pose[:2])
                if static_clearance(world, p[0], p[1]) >= ROBOT_RADIUS + 1e-3:
                    lo = mid
                else:
                    hi = mid
            new[:2] = self.pose[:2] + lo * (new[:2] - self.pose[:2])
            v = v * lo
            self.v = 0.0
        self.pose = new
        return v, w


def static_clearance(world, x, y):
    d = [point_segment_distance(x, y, world.walls).min()]
    if len(world.glass):
        d.append(point_segment_distance(x, y, world.glass).min())
    if world.obstacles:
        d.append(point_box_distance(x, y, world.obstacle_boxes()).min())
    return float(min(d))


def contacts(world, pose, actors_now):
    """Set of object ids the robot currently touches (walls, glass, obstacles, people)."""
    ids = set()
    x, y = pose[0], pose[1]
    r = ROBOT_RADIUS + 0.005
    for k in np.flatnonzero(point_segment_distance(x, y, world.walls) < r):
        ids.add(("wall", int(k)))
    if len(world.glass):
        for k in np.flatnonzero(point_segment_distance(x, y, world.glass) < r):
            ids.add(("glass", int(k)))
    if world.obstacles:
        for k in np.flatnonzero(point_box_distance(x, y, world.obstacle_boxes()) < r):
            ids.add((world.obstacles[k].kind, int(k)))
    for k, a in enumerate(actors_now):
        if np.hypot(a[0] - x, a[1] - y) < ROBOT_RADIUS + world.actors[k].radius:
            ids.add(("person", k))
    return ids


def run_episode(world_name, config, seed, controller="dwb", nav=NavParams(), record=False):
    cfg = CONFIGS[config] if isinstance(config, str) else config
    world = make_world(world_name, seed)
    ss = np.random.SeedSequence([seed, 11])
    r_phys, r_sens, r_amcl, r_ctrl = (np.random.default_rng(s) for s in ss.spawn(4))
    limits = RobotLimits()

    robot = Robot(world.start, limits)
    lidar, camera = Lidar(), CameraDetector()
    enc, gyro = WheelEncoders(r_sens), Gyro(r_sens)
    odom = Ekf(cfg.ekf_params) if cfg.use_ekf else WheelOdometry()
    init = world.start + np.array([r_amcl.normal(0, 0.05), r_amcl.normal(0, 0.05), r_amcl.normal(0, 0.02)])
    amcl = Amcl(world, init, r_amcl)
    monitor = LocalizationMonitor()
    cp = CostmapParams(use_camera=cfg.use_camera,
                       classes={**STATIC_CLASSES, "person": PERSON_AWARE if cfg.people_aware else PERSON_PLAIN})
    costmap = Costmap(world, cp)
    ctrl = MppiController(limits, rng=r_ctrl) if controller == "mppi" else DwbController(limits)

    t = 0.0
    path = None
    last_plan = -1e9
    last_costmap = -1e9
    progress_anchor, progress_t = world.start[:2].copy(), 0.0
    stuck_since = None
    recoveries = 0
    recovery_queue = []
    recovery_cycle = ["clear", "spin", "wait", "backup"]
    pending_dets = []
    in_contact = set()
    collisions = {"static": 0, "people": 0}
    min_clear_people = np.inf
    intrusion_s = 0.0
    true_len = 0.0
    err_sq = []
    nav_ms, layer_ms = [], []
    kidnapped = False
    relocalizing = None
    reloc_time = None
    ok_since = None
    status = "timeout"
    trace = []
    actor_trace, cost_trace = [], []

    ang, rng_scan = lidar.scan(world, robot.pose, world.actor_states(0.0), r_sens)
    amcl.update(odom.pose, ang, rng_scan)
    belief = amcl.belief(odom.pose)

    while t < world.timeout:
        tick = time.perf_counter()
        # ---------------- navigation stack (belief-space) ----------------
        v_cmd = w_cmd = 0.0
        if np.hypot(*(belief[:2] - world.goal)) < nav.goal_tolerance:
            status = "reached"
            break
        if relocalizing:
            w_cmd = 0.6                  # spin in place until the scan matches the map again
            if amcl.match_ratio(belief, ang, rng_scan) >= monitor.threshold + 0.2 or t - relocalizing > 20.0:
                relocalizing = None
                path = None
                ctrl.reset()
                progress_anchor, progress_t = belief[:2].copy(), t
        elif recovery_queue:
            action = recovery_queue[0]
            kind, remaining = action
            if kind == "spin":
                w_cmd = 1.0
                action[1] -= DT
            elif kind == "backup":
                v_cmd = -0.15
                action[1] -= DT
            elif kind == "wait":
                action[1] -= DT
            if action[1] <= 0:
                recovery_queue.pop(0)
                path = None
                ctrl.reset()
                progress_anchor, progress_t = belief[:2].copy(), t
        else:
            if t - last_costmap >= nav.costmap_period - 1e-9:
                costmap.update(t)
                last_costmap = t
            if path is None or t - last_plan >= nav.replan_period - 1e-9 or path_blocked(costmap, path, belief):
                path = plan(costmap, belief, world.goal)
                last_plan = t
            cmd = ctrl.compute(belief, robot.v, robot.w, path, costmap) if path is not None else None
            if cmd is None:
                stuck_since = t if stuck_since is None else stuck_since
            else:
                stuck_since = None
                v_cmd, w_cmd = cmd
            if np.hypot(*(belief[:2] - progress_anchor)) > nav.progress_radius:
                progress_anchor, progress_t = belief[:2].copy(), t
            failed = (stuck_since is not None and t - stuck_since >= nav.stuck_time) or (t - progress_t > nav.progress_time)
            if failed:
                if recoveries >= nav.max_recoveries:
                    status = "aborted"
                    break
                kind = recovery_cycle[recoveries % len(recovery_cycle)]
                recoveries += 1
                stuck_since = None
                if kind == "clear":
                    costmap.clear()
                    path = None
                    progress_anchor, progress_t = belief[:2].copy(), t
                else:
                    recovery_queue.append([kind, {"spin": np.pi / 2, "wait": 3.0, "backup": 2.0}[kind]])
        nav_ms.append((time.perf_counter() - tick) * 1e3)

        # ---------------- world ----------------
        prev = robot.pose.copy()
        v_true, w_true = robot.step(v_cmd, w_cmd, world, None, r_phys)
        true_len += np.hypot(*(robot.pose[:2] - prev[:2]))
        t += DT
        if world.kidnap and not kidnapped and t >= world.kidnap[0]:
            robot.pose = world.kidnap[1].copy()
            robot.v = robot.w = 0.0
            kidnapped = True
            ok_since = None
        world.step_actors(DT, robot.pose[:2])
        actors_now = world.actor_states(t)

        # ---------------- sensing + estimation ----------------
        tick = time.perf_counter()
        vm, wm = enc.measure(v_true, w_true, DT, r_sens)
        gm = gyro.measure(w_true, DT, r_sens)
        odom_pose = odom.update(vm, wm, DT, gm if cfg.use_ekf else None)
        ang, rng_scan = lidar.scan(world, robot.pose, actors_now, r_sens)
        amcl.update(odom_pose, ang, rng_scan)
        belief = amcl.belief(odom_pose)
        if monitor.step(t, amcl, belief, ang, rng_scan):
            costmap.clear()
            path = None
            relocalizing = t
        costmap.update_obstacles(belief, ang, rng_scan)
        if cfg.use_camera:
            t0 = time.perf_counter()
            cam_true, dets = camera.detect(world, robot.pose, actors_now, r_sens)
            cam_belief = compose(belief, np.array([camera.p.mount_x, 0.0, 0.0]))
            pending_dets.append((t + camera.p.latency, cam_belief, dets, t))
            while pending_dets and pending_dets[0][0] <= t + 1e-9:
                _, cb, d, stamp = pending_dets.pop(0)
                costmap.add_detections(cb, d, stamp)
            layer_ms.append((time.perf_counter() - t0) * 1e3)
        nav_ms[-1] += (time.perf_counter() - tick) * 1e3

        # ---------------- metrics (ground truth) ----------------
        now_contacts = contacts(world, robot.pose, actors_now)
        for c in now_contacts - in_contact:
            collisions["people" if c[0] == "person" else "static"] += 1
        in_contact = now_contacts
        if len(actors_now):
            d = np.hypot(actors_now[:, 0] - robot.pose[0], actors_now[:, 1] - robot.pose[1])
            gap = float(d.min() - ROBOT_RADIUS - world.actors[0].radius)
            min_clear_people = min(min_clear_people, gap)
            if gap < PERSONAL_SPACE:
                intrusion_s += DT
        e = np.hypot(*(belief[:2] - robot.pose[:2]))
        err_sq.append(e * e)
        if kidnapped and reloc_time is None:
            if e < 0.5:
                ok_since = t if ok_since is None else ok_since
                if t - ok_since >= 2.0:
                    reloc_time = ok_since - world.kidnap[0]
            else:
                ok_since = None
        if record:
            trace.append((t, *robot.pose, *belief, v_cmd, w_cmd))
            actor_trace.append(actors_now[:, :2].copy() if len(actors_now) else np.zeros((0, 2)))
            if len(trace) % 5 == 1:
                cost_trace.append((len(trace) - 1, costmap.master.copy(), None if path is None else path.copy()))

    true_err = float(np.hypot(*(robot.pose[:2] - world.goal)))
    success = status == "reached" and true_err < nav.success_radius
    res = {
        "world": world_name, "config": cfg.name, "controller": controller, "seed": seed,
        "success": int(success), "status": status,
        "time_to_goal": round(t, 2) if success else float("nan"),
        "path_length": round(true_len, 3),
        "collisions": collisions["static"] + collisions["people"],
        "collisions_static": collisions["static"], "collisions_people": collisions["people"],
        "min_clearance_people": round(min_clear_people, 3) if np.isfinite(min_clear_people) else float("nan"),
        "personal_space_s": round(intrusion_s, 2) if len(world.actors) else float("nan"),
        "recoveries": recoveries,
        "ate_rmse": round(float(np.sqrt(np.mean(err_sq))), 4) if err_sq else float("nan"),
        "final_goal_error": round(true_err, 3),
        "nav_cpu_ms": round(float(np.mean(nav_ms)), 3),
        "perception_layer_ms": round(float(np.mean(layer_ms)), 4) if layer_ms else float("nan"),
        "global_relocalizations": monitor.triggers,
        "relocalization_time": round(reloc_time, 2) if reloc_time is not None else float("nan"),
        "sim_time": round(t, 2),
    }
    if record:
        res["trace"] = np.array(trace)
        res["actor_trace"] = actor_trace
        res["cost_trace"] = cost_trace
        res["costmap_origin"] = costmap.origin
        res["costmap_res"] = costmap.p.resolution
        res["world_obj"] = world
    return res
