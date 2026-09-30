#!/usr/bin/env python3
"""Stamps honest covariances onto wheel odometry and IMU before the EKF, and
removes the gyro bias estimated while the robot stands still at start-up.

Gazebo's diff-drive plugin reports near-zero twist covariance and the IMU
sensor reports none; robot_localization weights sources by these numbers,
so leaving them alone is the "untuned EKF" failure mode (see ekf.yaml).
"""
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class CovarianceRelay(Node):
    def __init__(self):
        super().__init__('covariance_relay')
        self.var_vx = self.declare_parameter('wheel_vx_variance', 0.0004).value
        self.var_wz = self.declare_parameter('wheel_vyaw_variance', 0.0064).value
        self.var_gz = self.declare_parameter('gyro_vyaw_variance', 0.000025).value
        self.calib_time = self.declare_parameter('gyro_calibration_time', 2.0).value
        self.pub_odom = self.create_publisher(Odometry, '/odom/wheel_cov', 10)
        self.pub_imu = self.create_publisher(Imu, '/imu/data_cov', qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom/wheel', self.on_odom, 10)
        self.create_subscription(Imu, '/imu/data', self.on_imu, qos_profile_sensor_data)
        self.samples = []
        self.bias = None
        self.t0 = None

    def on_odom(self, msg):
        cov = np.zeros(36)
        cov[0], cov[7], cov[35] = self.var_vx, self.var_vx, self.var_wz
        cov[14] = cov[21] = cov[28] = 1e3   # unused dimensions
        msg.twist.covariance = cov.tolist()
        self.pub_odom.publish(msg)

    def on_imu(self, msg):
        t = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        if self.bias is None:
            self.t0 = t if self.t0 is None else self.t0
            self.samples.append(msg.angular_velocity.z)
            if t - self.t0 < self.calib_time:
                return
            self.bias = float(np.mean(self.samples))
            self.get_logger().info(f'gyro z bias {self.bias:.5f} rad/s from {len(self.samples)} samples')
        msg.angular_velocity.z -= self.bias
        cov = [1e3, 0.0, 0.0, 0.0, 1e3, 0.0, 0.0, 0.0, self.var_gz]
        msg.angular_velocity_covariance = cov
        msg.orientation_covariance[0] = -1.0     # orientation not provided
        self.pub_imu.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(CovarianceRelay())


if __name__ == '__main__':
    main()
