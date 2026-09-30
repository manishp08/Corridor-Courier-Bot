"""Nav2 servers for one ablation config and controller.

  ros2 launch cc_navigation navigation.launch.py ablation:=D controller:=mppi

Every node gets [nav2_params.yaml, <costmap overlay>, <controller overlay>];
later files override earlier ones, so configs differ only in the overlays.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

COSTMAPS = {"A": "costmaps_lidar.yaml", "B": "costmaps_lidar.yaml",
            "C": "costmaps_camera.yaml", "D": "costmaps_people.yaml"}


def _nodes(context):
    share = get_package_share_directory("cc_navigation")
    cfg = os.path.join(share, "config")
    ablation = context.launch_configurations["ablation"].upper()
    controller = context.launch_configurations["controller"].lower()
    odom_topic = "/odom/wheel" if ablation == "A" else "/odometry/filtered"
    params = [os.path.join(cfg, "nav2_params.yaml"),
              os.path.join(cfg, COSTMAPS[ablation]),
              os.path.join(cfg, f"controller_{controller}.yaml"),
              {"use_sim_time": True}]
    bt = {"default_nav_to_pose_bt_xml": os.path.join(share, "behavior_trees", "cc_navigate.xml"),
          "odom_topic": odom_topic}
    return [
        Node(package="nav2_controller", executable="controller_server", output="screen",
             parameters=params, remappings=[("cmd_vel", "cmd_vel_nav")]),
        Node(package="nav2_planner", executable="planner_server", name="planner_server", output="screen",
             parameters=params),
        Node(package="nav2_behaviors", executable="behavior_server", name="behavior_server", output="screen",
             parameters=params, remappings=[("cmd_vel", "cmd_vel_nav")]),
        Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator", output="screen",
             parameters=params + [bt]),
        Node(package="nav2_velocity_smoother", executable="velocity_smoother", name="velocity_smoother",
             output="screen", parameters=params + [{"odom_topic": odom_topic}],
             remappings=[("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel_navigation")]),
        # Relocalization spin (localization_monitor) overrides navigation.
        Node(package="twist_mux", executable="twist_mux", name="twist_mux", output="screen",
             parameters=[os.path.join(cfg, "twist_mux.yaml"), {"use_sim_time": True}],
             remappings=[("cmd_vel_out", "cmd_vel")]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_navigation",
             output="screen", parameters=params),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("ablation", default_value="D", description="A | B | C | D"),
        DeclareLaunchArgument("controller", default_value="dwb", description="dwb | mppi"),
        OpaqueFunction(function=_nodes),
    ])
