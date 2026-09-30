#!/usr/bin/env python3
"""Kidnapped-robot detector (same logic as sim/ccsim/localization.py LocalizationMonitor).

Projects each scan from the AMCL pose onto the map's distance field; if the
fraction of beams landing within `tolerance` of an occupied cell stays below
`threshold` for `hold_time`, it calls /reinitialize_global_localization,
spins the robot in place (forcing filter updates via /request_nomotion_update)
until the match recovers, and publishes /localization/lost for the harness.
"""
import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from scipy import ndimage
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import Empty


def yaw_of(q):
    return np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class LocalizationMonitor(Node):
    def __init__(self):
        super().__init__('localization_monitor')
        self.threshold = self.declare_parameter('threshold', 0.35).value
        self.recovered = self.declare_parameter('recovered_threshold', 0.55).value
        self.hold_time = self.declare_parameter('hold_time', 3.0).value
        self.cooldown = self.declare_parameter('cooldown', 10.0).value
        self.tol = self.declare_parameter('tolerance', 0.2).value
        self.spin_speed = self.declare_parameter('spin_speed', 0.6).value
        self.field = None
        self.pose = None
        self.bad_since = None
        self.last_trigger = -1e9
        self.relocalizing_since = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_pose, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.lost_pub = self.create_publisher(Bool, '/localization/lost', 10)
        # Published on a mux-priority topic so it overrides Nav2 while relocalizing.
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_relocalize', 10)
        self.global_srv = self.create_client(Empty, '/reinitialize_global_localization')
        self.nomotion_srv = self.create_client(Empty, '/request_nomotion_update')

    def on_map(self, msg):
        occ = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width) > 50
        self.res = msg.info.resolution
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.field = ndimage.distance_transform_edt(~occ) * self.res

    def on_pose(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def match_ratio(self, scan):
        r = np.asarray(scan.ranges)
        a = scan.angle_min + np.arange(len(r)) * scan.angle_increment
        ok = np.isfinite(r) & (r > scan.range_min) & (r < scan.range_max)
        if ok.sum() < 10:
            return 1.0
        x, y, th = self.pose
        ex = x + r[ok] * np.cos(th + a[ok])
        ey = y + r[ok] * np.sin(th + a[ok])
        i = ((ex - self.origin[0]) / self.res).astype(int)
        j = ((ey - self.origin[1]) / self.res).astype(int)
        h, w = self.field.shape
        inside = (i >= 0) & (i < w) & (j >= 0) & (j < h)
        d = np.full(len(i), np.inf)
        d[inside] = self.field[j[inside], i[inside]]
        return float(np.mean(d < self.tol))

    def on_scan(self, scan):
        if self.field is None or self.pose is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        ratio = self.match_ratio(scan)
        if self.relocalizing_since is not None:
            if ratio >= self.recovered or now - self.relocalizing_since > 20.0:
                self.relocalizing_since = None
                self.cmd_pub.publish(Twist())
                self.lost_pub.publish(Bool(data=False))
                self.get_logger().info(f'relocalized (match {ratio:.2f})')
            else:
                t = Twist()
                t.angular.z = self.spin_speed
                self.cmd_pub.publish(t)
                if self.nomotion_srv.service_is_ready():
                    self.nomotion_srv.call_async(Empty.Request())
            return
        if ratio >= self.threshold:
            self.bad_since = None
            return
        self.bad_since = now if self.bad_since is None else self.bad_since
        if now - self.bad_since >= self.hold_time and now - self.last_trigger >= self.cooldown:
            self.get_logger().warn(f'scan/map match {ratio:.2f} < {self.threshold}: global relocalization')
            if self.global_srv.service_is_ready():
                self.global_srv.call_async(Empty.Request())
            self.last_trigger = now
            self.bad_since = None
            self.relocalizing_since = now
            self.lost_pub.publish(Bool(data=True))


def main():
    rclpy.init()
    rclpy.spin(LocalizationMonitor())


if __name__ == '__main__':
    main()
