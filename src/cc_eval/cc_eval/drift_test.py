"""Milestone 3 in Gazebo: drive a 20 m square open-loop and measure drift.

  ros2 launch cc_gazebo sim.launch.py world:=empty_corridor    # any open world
  ros2 launch cc_localization localization.launch.py ekf:=true mode:=none
  ros2 run cc_eval drift_test --ros-args -p use_sim_time:=true -p side:=5.0

Records ground truth, raw wheel odometry and the EKF output, aligns each to
the start pose, and prints final / max position error and yaw error for
wheel-only vs EKF. Same protocol as sim/ccsim/drift.py. Needs ~9 m x 9 m of
free floor: use the kidnapped world's outer loop or an empty world.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def pose_of(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    return np.array([p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))])


def relative(poses):
    """Express a trajectory relative to its first pose."""
    p0 = poses[0]
    c, s = math.cos(-p0[2]), math.sin(-p0[2])
    d = poses[:, :2] - p0[:2]
    return np.column_stack([c * d[:, 0] - s * d[:, 1], s * d[:, 0] + c * d[:, 1],
                            (poses[:, 2] - p0[2] + np.pi) % (2 * np.pi) - np.pi])


class DriftTest(Node):
    def __init__(self):
        super().__init__("drift_test")
        self.side = self.declare_parameter("side", 5.0).value
        self.v = self.declare_parameter("speed", 0.4).value
        self.w = self.declare_parameter("turn_rate", 0.6).value
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.latest = {}
        self.log = {k: [] for k in ("gt", "wheel", "ekf")}
        for key, topic in (("gt", "/ground_truth/odom"), ("wheel", "/odom/wheel"), ("ekf", "/odometry/filtered")):
            self.create_subscription(Odometry, topic, lambda m, k=key: self.latest.__setitem__(k, pose_of(m)), 50)

    def drive(self):
        plan = []
        for _ in range(4):
            plan += [(self.v, 0.0, self.side / self.v), (0.0, self.w, (math.pi / 2) / self.w)]
        plan.append((0.0, 0.0, 2.0))
        for v, w, dur in plan:
            t_end = self.get_clock().now().nanoseconds * 1e-9 + dur
            while self.get_clock().now().nanoseconds * 1e-9 < t_end:
                msg = Twist()
                msg.linear.x, msg.angular.z = v, w
                self.pub.publish(msg)
                rclpy.spin_once(self, timeout_sec=0.02)
                if all(k in self.latest for k in ("gt", "wheel")):
                    for k in self.log:
                        if k in self.latest:
                            self.log[k].append(self.latest[k].copy())

    def report(self):
        gt = relative(np.array(self.log["gt"]))
        for k in ("wheel", "ekf"):
            if not self.log[k]:
                print(f"{k}: no data")
                continue
            est = relative(np.array(self.log[k]))
            n = min(len(gt), len(est))
            err = np.hypot(est[:n, 0] - gt[:n, 0], est[:n, 1] - gt[:n, 1])
            yaw = abs((est[n - 1, 2] - gt[n - 1, 2] + np.pi) % (2 * np.pi) - np.pi)
            print(f"{k:6s} final {err[-1]:.3f} m  max {err.max():.3f} m  yaw {math.degrees(yaw):.2f} deg")


def main():
    rclpy.init()
    n = DriftTest()
    while rclpy.ok() and n.get_clock().now().nanoseconds == 0:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.drive()
    n.report()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
