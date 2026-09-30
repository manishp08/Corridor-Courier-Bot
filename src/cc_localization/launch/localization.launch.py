"""Localization for one ablation config.

  ros2 launch cc_localization localization.launch.py ekf:=true mode:=amcl world:=glass_wall

ekf:=true   -> covariance_relay + robot_localization EKF publish odom->base_footprint (configs B-D)
ekf:=false  -> the gz diff-drive TF is used directly (config A; sim.launch.py wheel_tf:=true)
mode:=slam  -> slam_toolbox mapping (milestone 4, map building)
mode:=amcl  -> map_server + AMCL on the saved map + kidnap monitor
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = get_package_share_directory('cc_localization')
    ekf_yaml = os.path.join(share, 'config', 'ekf.yaml')
    amcl_yaml = os.path.join(share, 'config', 'amcl.yaml')
    slam_yaml = os.path.join(share, 'config', 'slam_toolbox.yaml')
    map_yaml = PathJoinSubstitution([FindPackageShare('cc_localization'), 'maps',
                                     [LaunchConfiguration('world'), '.yaml']])
    sim = {'use_sim_time': True}
    return LaunchDescription([
        DeclareLaunchArgument('ekf', default_value='true'),
        DeclareLaunchArgument('mode', default_value='amcl'),
        DeclareLaunchArgument('world', default_value='empty_corridor'),
        Node(package='cc_localization', executable='covariance_relay.py', name='covariance_relay',
             parameters=[ekf_yaml, sim], condition=IfCondition(LaunchConfiguration('ekf'))),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             parameters=[ekf_yaml, sim], condition=IfCondition(LaunchConfiguration('ekf'))),
        Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
             parameters=[slam_yaml, sim], condition=LaunchConfigurationEquals('mode', 'slam')),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[{'yaml_filename': map_yaml}, sim], condition=LaunchConfigurationEquals('mode', 'amcl')),
        Node(package='nav2_amcl', executable='amcl', name='amcl',
             parameters=[amcl_yaml, sim], condition=LaunchConfigurationEquals('mode', 'amcl')),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_localization',
             parameters=[{'autostart': True, 'node_names': ['map_server', 'amcl']}, sim],
             condition=LaunchConfigurationEquals('mode', 'amcl')),
        Node(package='cc_localization', executable='localization_monitor.py', name='localization_monitor',
             parameters=[sim], condition=LaunchConfigurationEquals('mode', 'amcl')),
    ])
