"""Full stack for one (world, ablation config, controller):

  ros2 launch cc_navigation bringup.launch.py world:=glass_wall ablation:=C controller:=dwb

  A: gz diff-drive TF (wheel odometry)            + AMCL + LiDAR costmaps
  B: covariance_relay + EKF (wheel + IMU)         + AMCL + LiDAR costmaps
  C: B + detector + perception layer (people inflated like objects)
  D: C with people-aware inflation

use_sim_time is true in every node (mismatched sim time is the #1 cause of TF errors).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _stack(context):
    lc = context.launch_configurations
    world, ablation, controller = lc["world"], lc["ablation"].upper(), lc["controller"]
    pkg = get_package_share_directory

    def inc(p, f, **args):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(pkg(p), "launch", f)),
                                        launch_arguments=args.items())

    actions = [
        inc("cc_gazebo", "sim.launch.py", world=world, headless=lc["headless"],
            wheel_tf="true" if ablation == "A" else "false"),
        inc("cc_localization", "localization.launch.py", world=world, mode="amcl",
            ekf="false" if ablation == "A" else "true"),
        inc("cc_navigation", "navigation.launch.py", ablation=ablation, controller=controller),
    ]
    if ablation in ("C", "D"):
        actions.append(inc("cc_perception", "perception.launch.py", world=world, backend=lc["detector"],
                           model_path=lc["model_path"], seed=lc["seed"]))
    if lc["rviz"] == "true":
        actions.append(Node(package="rviz2", executable="rviz2", parameters=[{"use_sim_time": True}],
                            arguments=["-d", os.path.join(pkg("cc_description"), "rviz", "courier.rviz")]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="empty_corridor"),
        DeclareLaunchArgument("ablation", default_value="D"),
        DeclareLaunchArgument("controller", default_value="dwb"),
        DeclareLaunchArgument("detector", default_value="oracle", description="oracle | onnx"),
        DeclareLaunchArgument("model_path", default_value=""),
        DeclareLaunchArgument("seed", default_value="0"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        OpaqueFunction(function=_stack),
    ])
