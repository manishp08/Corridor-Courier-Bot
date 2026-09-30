"""Gazebo evaluation harness: runs ONE scripted episode against a running stack
and appends one row (same schema as the surrogate) to a CSV.

  ros2 run cc_eval harness --ros-args -p world:=glass_wall -p config:=C \
      -p controller:=dwb -p seed:=1000 -p out_csv:=results/gazebo/ablation.csv -p bag_dir:=bags

Ground truth comes from the gz OdometryPublisher (/ground_truth/odom) and the
world manifest (walls, glass, objects, actor scripts); the stack never sees
either. Recoveries come from NavigateToPose feedback. A rosbag and TUM
trajectories (for `evo_ape tum gt.tum est.tum`) are written per episode.
"""
import json
import math
import os
import signal
import subprocess
import time
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

from .metrics import ate_rmse, count_contacts, path_length
from .results_io import FIELDS, write_rows

ROBOT_RADIUS = 0.25
BAG_TOPICS = ["/tf", "/tf_static", "/scan", "/amcl_pose", "/ground_truth/odom", "/odometry/filtered",
              "/odom/wheel", "/imu/data", "/perception/detections", "/cmd_vel", "/plan",
              "/localization/lost", "/perception/latency_ms"]


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def seg_dist(p, segs):
    if len(segs) == 0:
        return np.full(0, np.inf)
    s = np.asarray(segs, float)
    e = s[:, 2:] - s[:, :2]
    t = np.clip(np.sum((p - s[:, :2]) * e, axis=1) / np.maximum(np.sum(e * e, axis=1), 1e-12), 0, 1)
    return np.linalg.norm(s[:, :2] + t[:, None] * e - p, axis=1)


def box_dist(p, o):
    c, s = math.cos(o["yaw"]), math.sin(o["yaw"])
    dx, dy = p[0] - o["x"], p[1] - o["y"]
    lx = abs(c * dx + s * dy) - o["size_x"] / 2
    ly = abs(-s * dx + c * dy) - o["size_y"] / 2
    return math.hypot(max(lx, 0), max(ly, 0))


class ActorScript:
    def __init__(self, spec):
        self.wp = np.asarray(spec["waypoints"], float)
        self.speed = spec["speed"]
        self.length = float(np.linalg.norm(self.wp[-1] - self.wp[0]))
        self.s0 = (spec["phase"] * self.speed) % (2 * self.length)
        self.radius = spec.get("radius", 0.25)

    def position(self, t):
        s = (self.s0 + self.speed * t) % (2 * self.length)
        if s > self.length:
            s = 2 * self.length - s
        return self.wp[0] + (self.wp[-1] - self.wp[0]) * s / max(self.length, 1e-9)


class Harness(BasicNavigator):
    def __init__(self):
        super().__init__("cc_eval_harness")
        p = lambda n, d: self.declare_parameter(n, d).value  # noqa: E731
        self.world, self.config, self.controller = p("world", "empty_corridor"), p("config", "D"), p("controller", "dwb")
        self.seed = p("seed", 1000)
        self.out_csv = p("out_csv", "results/gazebo/ablation.csv")
        self.bag_dir = p("bag_dir", "")
        manifest = os.path.join(get_package_share_directory("cc_gazebo"), "worlds", f"{self.world}.json")
        self.m = json.loads(Path(manifest).read_text())
        self.actors = [ActorScript(a) for a in self.m["actors"]]
        self.gt = None
        self.belief = None
        self.samples = []          # (t, gt_x, gt_y, gt_yaw, est_x, est_y, est_yaw)
        self.contacts = []
        self.min_clear = math.inf
        self.latency = []
        self.lost_events = 0
        self.create_subscription(Odometry, "/ground_truth/odom", self.on_gt, 20)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.on_belief, 10)
        self.create_subscription(Float64, "/perception/latency_ms", lambda m: self.latency.append(m.data), 10)

    def on_gt(self, msg):
        p = msg.pose.pose
        self.gt = (p.position.x, p.position.y, yaw_of(p.orientation))

    def on_belief(self, msg):
        p = msg.pose.pose
        self.belief = (p.position.x, p.position.y, yaw_of(p.orientation))

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def sample(self, t):
        if self.gt is None:
            return
        x, y = self.gt[0], self.gt[1]
        p = np.array([x, y])
        ids = set()
        r = ROBOT_RADIUS + 0.005
        ids |= {("wall", int(k)) for k in np.flatnonzero(seg_dist(p, self.m["walls"]) < r)}
        ids |= {("glass", int(k)) for k in np.flatnonzero(seg_dist(p, self.m["glass"]) < r)}
        ids |= {(o["kind"], k) for k, o in enumerate(self.m["objects"]) if box_dist(p, o) < r}
        for k, a in enumerate(self.actors):
            d = float(np.linalg.norm(a.position(t) - p)) - ROBOT_RADIUS - a.radius
            self.min_clear = min(self.min_clear, d)
            if d < 0:
                ids.add(("person", k))
        self.contacts.append(ids)
        if self.belief is not None:
            self.samples.append((t, *self.gt, *self.belief))

    def teleport(self, pose):
        x, y, yaw = pose
        req = (f'name: "courier", position {{x: {x} y: {y} z: 0.02}}, '
               f'orientation {{z: {math.sin(yaw / 2)} w: {math.cos(yaw / 2)}}}')
        subprocess.run(["gz", "service", "-s", f"/world/{self.world}/set_pose", "--reqtype", "gz.msgs.Pose",
                        "--reptype", "gz.msgs.Boolean", "--timeout", "2000", "--req", req], check=False)

    def run(self):
        rng = np.random.default_rng([self.seed, 5])
        start, goal = self.m["start"], self.m["goal"]
        init = PoseStamped()
        init.header.frame_id = "map"
        init.header.stamp = self.get_clock().now().to_msg()
        init.pose.position.x = start[0] + rng.normal(0, 0.05)
        init.pose.position.y = start[1] + rng.normal(0, 0.05)
        yaw = start[2] + rng.normal(0, 0.02)
        init.pose.orientation.z, init.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.setInitialPose(init)
        self.waitUntilNav2Active(localizer="amcl")

        bag = None
        if self.bag_dir:
            out = Path(self.bag_dir) / f"{self.world}_{self.config}_{self.controller}_{self.seed}"
            bag = subprocess.Popen(["ros2", "bag", "record", "-o", str(out), *BAG_TOPICS],
                                   start_new_session=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        g = PoseStamped()
        g.header.frame_id = "map"
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose.position.x, g.pose.position.y = goal[0], goal[1]
        g.pose.orientation.w = 1.0
        t0 = self.now_s()
        self.goToPose(g)
        kidnap = self.m.get("kidnap")
        kidnapped = False
        recoveries = 0
        timed_out = False
        while not self.isTaskComplete():          # spins this node ~10 Hz
            t = self.now_s() - t0
            self.sample(t)
            fb = self.getFeedback()
            if fb is not None:
                recoveries = max(recoveries, fb.number_of_recoveries)
            if kidnap and not kidnapped and t >= kidnap["time"]:
                self.teleport(kidnap["pose"])
                kidnapped = True
            if t > self.m["timeout"]:
                self.cancelTask()
                timed_out = True
                break
        result = self.getResult()
        t_end = self.now_s() - t0
        if bag is not None:
            os.killpg(bag.pid, signal.SIGINT)
            bag.wait(timeout=10)

        S = np.array(self.samples) if self.samples else np.zeros((0, 7))
        final_err = float(np.hypot(self.gt[0] - goal[0], self.gt[1] - goal[1])) if self.gt else math.nan
        success = (result == TaskResult.SUCCEEDED) and final_err < 0.5 and not timed_out
        contact_events = count_contacts(self.contacts)
        people = count_contacts([{c for c in s if c[0] == "person"} for s in self.contacts])
        reloc = math.nan
        if kidnap and len(S):
            err = np.hypot(S[:, 1] - S[:, 4], S[:, 2] - S[:, 5])
            after = S[:, 0] >= kidnap["time"]
            ok = after & (err < 0.5)
            for i in np.flatnonzero(ok):
                j = np.searchsorted(S[:, 0], S[i, 0] + 2.0)
                if j <= len(S) and np.all(ok[i:j]):
                    reloc = float(S[i, 0] - kidnap["time"])
                    break
        row = {
            "world": self.world, "config": self.config, "controller": self.controller, "seed": self.seed,
            "success": int(success), "status": "reached" if result == TaskResult.SUCCEEDED else ("timeout" if timed_out else "aborted"),
            "time_to_goal": round(t_end, 2) if success else math.nan,
            "path_length": round(path_length(S[:, 1:3]), 3) if len(S) else math.nan,
            "collisions": contact_events, "collisions_static": contact_events - people, "collisions_people": people,
            "min_clearance_people": round(self.min_clear, 3) if self.actors else math.nan,
            "recoveries": recoveries,
            "ate_rmse": round(ate_rmse(S[:, 4:6], S[:, 1:3]), 4) if len(S) else math.nan,
            "final_goal_error": round(final_err, 3),
            "nav_cpu_ms": math.nan,
            "perception_layer_ms": round(float(np.mean(self.latency)), 3) if self.latency else math.nan,
            "global_relocalizations": math.nan,
            "relocalization_time": round(reloc, 2) if not math.isnan(reloc) else math.nan,
            "sim_time": round(t_end, 2),
        }
        Path(self.out_csv).parent.mkdir(parents=True, exist_ok=True)
        write_rows(self.out_csv, [row], mode="a" if Path(self.out_csv).exists() else "w")
        if self.bag_dir and len(S):
            base = Path(self.bag_dir) / f"{self.world}_{self.config}_{self.controller}_{self.seed}"
            base.mkdir(parents=True, exist_ok=True)
            for name, cols in (("gt.tum", (1, 2, 3)), ("est.tum", (4, 5, 6))):
                with open(base / name, "w") as f:
                    for r in S:
                        th = r[cols[2]]
                        f.write(f"{r[0]:.3f} {r[cols[0]]:.4f} {r[cols[1]]:.4f} 0 0 0 {math.sin(th / 2):.6f} {math.cos(th / 2):.6f}\n")
        self.get_logger().info(f"episode done: {row}")
        return row


def main():
    rclpy.init()
    h = Harness()
    try:
        h.run()
    finally:
        h.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
