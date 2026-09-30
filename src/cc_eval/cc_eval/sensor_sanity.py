"""Milestone 2 acceptance test: sensor rates, frame ids and timestamp lag vs /clock.

  ros2 run cc_eval sensor_sanity --ros-args -p use_sim_time:=true -p duration:=20.0

Fails (exit 1) if a topic is missing, slower than 80% of its nominal rate,
uses an unexpected frame, or is stamped more than max_lag behind sim time.
A stamp lag of many seconds almost always means some node is not on sim time.
"""
import sys

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan

CHECKS = [
    # topic, type, nominal Hz, expected frame
    ("/scan", LaserScan, 10.0, "lidar_link"),
    ("/imu/data", Imu, 100.0, "imu_link"),
    ("/odom/wheel", Odometry, 50.0, "odom"),
    ("/camera/image", Image, 15.0, "camera_optical_frame"),
    ("/camera/depth_image", Image, 15.0, "camera_optical_frame"),
    ("/camera/camera_info", CameraInfo, 15.0, "camera_optical_frame"),
]


class SensorSanity(Node):
    def __init__(self):
        super().__init__("sensor_sanity")
        self.duration = self.declare_parameter("duration", 20.0).value
        self.max_lag = self.declare_parameter("max_lag", 0.2).value
        self.stats = {t: {"stamps": [], "lags": [], "frames": set()} for t, *_ in CHECKS}
        for topic, typ, *_ in CHECKS:
            self.create_subscription(typ, topic, lambda m, t=topic: self.on_msg(t, m), qos_profile_sensor_data)

    def on_msg(self, topic, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        st = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        s = self.stats[topic]
        s["stamps"].append(st)
        s["lags"].append(now - st)
        s["frames"].add(msg.header.frame_id)

    def report(self):
        ok_all = True
        print(f"{'topic':24s} {'rate Hz':>8s} {'nominal':>8s} {'lag ms (p95)':>13s}  frame")
        for topic, _, hz, frame in CHECKS:
            s = self.stats[topic]
            if len(s["stamps"]) < 3:
                print(f"{topic:24s} MISSING")
                ok_all = False
                continue
            dt = np.diff(s["stamps"])
            rate = 1.0 / np.median(dt[dt > 0]) if np.any(dt > 0) else 0.0
            lag = float(np.percentile(s["lags"], 95))
            frames = ",".join(sorted(s["frames"]))
            ok = rate >= 0.8 * hz and lag <= self.max_lag and s["frames"] == {frame}
            ok_all &= ok
            print(f"{topic:24s} {rate:8.1f} {hz:8.1f} {1000 * lag:13.1f}  {frames} {'OK' if ok else 'FAIL'}")
        return ok_all


def main():
    rclpy.init()
    n = SensorSanity()
    clock_start = None
    while rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.1)
        t = n.get_clock().now().nanoseconds * 1e-9
        if t > 0 and clock_start is None:
            clock_start = t
        if clock_start is not None and t - clock_start >= n.duration:
            break
    ok = n.report()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
