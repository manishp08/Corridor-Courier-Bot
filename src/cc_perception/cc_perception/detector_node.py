"""ROS 2 detector node: RGB-D -> vision_msgs/Detection3DArray on /perception/detections.

backend:=onnx    YOLOv8 ONNX on /camera/image + /camera/depth_image (real pipeline)
backend:=oracle  ground-truth objects from the world manifest with the
                 surrogate's noise model (for Gazebo runs without a fine-tuned model)

Detections carry the *capture* timestamp so the costmap layer transforms
them with TF at the time the image was taken. The node also publishes its
processing latency on /perception/latency_ms for the eval harness.
"""
import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Quaternion
from message_filters import ApproximateTimeSynchronizer, Subscriber
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float64
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

from .projection import Detection2D, Intrinsics, project


def yaw_quat(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))


def image_to_numpy(msg):
    if msg.encoding in ("rgb8", "bgr8"):
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        return img[..., ::-1].copy() if msg.encoding == "bgr8" else img
    if msg.encoding == "32FC1":
        return np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width)
    if msg.encoding == "16UC1":
        return np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.width).astype(np.float32) / 1000.0
    raise ValueError(f"unsupported encoding {msg.encoding}")


class DetectorNode(Node):
    def __init__(self):
        super().__init__("cc_detector")
        self.backend = self.declare_parameter("backend", "oracle").value
        self.model_path = self.declare_parameter("model_path", "").value
        self.manifest_path = self.declare_parameter("manifest", "").value
        self.cam_height = self.declare_parameter("camera_height", 0.45).value
        self.cam_pitch = self.declare_parameter("camera_pitch", 0.2618).value
        self.rate = self.declare_parameter("oracle_rate", 10.0).value
        self.pub = self.create_publisher(Detection3DArray, "/perception/detections", qos_profile_sensor_data)
        self.lat_pub = self.create_publisher(Float64, "/perception/latency_ms", 10)
        self.intr = None
        if self.backend == "onnx":
            from .onnx_detector import OnnxDetector
            self.detector = OnnxDetector(self.model_path)
            self.create_subscription(CameraInfo, "/camera/camera_info", self.on_info, 10)
            rgb = Subscriber(self, Image, "/camera/image", qos_profile=qos_profile_sensor_data)
            depth = Subscriber(self, Image, "/camera/depth_image", qos_profile=qos_profile_sensor_data)
            self.sync = ApproximateTimeSynchronizer([rgb, depth], queue_size=5, slop=0.05)
            self.sync.registerCallback(self.on_rgbd)
        elif self.backend == "oracle":
            from .oracle import OracleDetector
            with open(self.manifest_path) as f:
                self.oracle = OracleDetector(json.load(f), seed=self.declare_parameter("seed", 0).value)
            self.gt = None
            self.create_subscription(Odometry, "/ground_truth/odom", self.on_gt, 10)
            self.create_timer(1.0 / self.rate, self.on_oracle_tick)
        else:
            raise ValueError(f"unknown backend {self.backend}")
        self.get_logger().info(f"detector backend: {self.backend}")

    # ------------------------------------------------------------------ onnx
    def on_info(self, msg):
        self.intr = Intrinsics.from_k(msg.k)

    def on_rgbd(self, rgb_msg, depth_msg):
        if self.intr is None:
            return
        t0 = self.get_clock().now()
        rgb = image_to_numpy(rgb_msg)
        depth = image_to_numpy(depth_msg)
        out = Detection3DArray()
        out.header = rgb_msg.header                      # capture stamp, camera_optical_frame
        for cls, score, x1, y1, x2, y2 in self.detector(rgb):
            obj = project(Detection2D(cls, score, x1, y1, x2, y2), depth, self.intr, self.cam_height, self.cam_pitch)
            if obj is None:
                continue
            d = Detection3D()
            d.header = rgb_msg.header
            h = ObjectHypothesisWithPose()
            h.hypothesis.class_id = obj.cls
            h.hypothesis.score = obj.score
            d.results.append(h)
            d.bbox.center.position.x, d.bbox.center.position.y, d.bbox.center.position.z = obj.x, obj.y, obj.z
            d.bbox.center.orientation.w = 1.0            # size.x lies along optical x (horizontal)
            d.bbox.size.x, d.bbox.size.y, d.bbox.size.z = obj.size_x, obj.size_y, 0.5
            out.detections.append(d)
        self.pub.publish(out)
        self.lat_pub.publish(Float64(data=(self.get_clock().now() - t0).nanoseconds * 1e-6))

    # ---------------------------------------------------------------- oracle
    def on_gt(self, msg):
        p = msg.pose.pose
        q = p.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.gt = (p.position.x, p.position.y, yaw)

    def on_oracle_tick(self):
        if self.gt is None:
            return
        now = self.get_clock().now()
        out = Detection3DArray()
        out.header.stamp = now.to_msg()
        out.header.frame_id = "base_footprint"
        for det in self.oracle.detect(self.gt, now.nanoseconds * 1e-9):
            d = Detection3D()
            d.header = out.header
            h = ObjectHypothesisWithPose()
            h.hypothesis.class_id = det["cls"]
            h.hypothesis.score = det["score"]
            d.results.append(h)
            d.bbox.center.position.x, d.bbox.center.position.y = det["x"], det["y"]
            d.bbox.center.orientation = yaw_quat(det["yaw"])
            d.bbox.size.x, d.bbox.size.y, d.bbox.size.z = det["size_x"], det["size_y"], 0.5
            out.detections.append(d)
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(DetectorNode())


if __name__ == "__main__":
    main()
