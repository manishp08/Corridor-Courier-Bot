"""Gazebo Harmonic + robot + bridges.

  ros2 launch cc_gazebo sim.launch.py world:=glass_wall wheel_tf:=true

wheel_tf:=true bridges the diff-drive plugin's odom->base_footprint TF
(ablation config A). With the EKF (configs B-D) leave it false so exactly one
node publishes that transform.
"""
import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _spawn(context):
    share = get_package_share_directory('cc_gazebo')
    world = LaunchConfiguration('world').perform(context)
    with open(os.path.join(share, 'worlds', f'{world}.json')) as f:
        start = json.load(f)['start']
    return [Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-name', 'courier', '-topic', 'robot_description',
                   '-x', str(start[0]), '-y', str(start[1]), '-z', '0.02', '-Y', str(start[2])],
        parameters=[{'use_sim_time': True}])]


def _gz(context):
    share = get_package_share_directory('cc_gazebo')
    world = LaunchConfiguration('world').perform(context)
    headless = LaunchConfiguration('headless').perform(context) == 'true'
    args = f"-r {'-s --headless-rendering ' if headless else ''}{os.path.join(share, 'worlds', world + '.sdf')}"
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': args, 'on_exit_shutdown': 'true'}.items())]


def generate_launch_description():
    share = get_package_share_directory('cc_gazebo')
    desc = os.path.join(get_package_share_directory('cc_description'), 'launch', 'description.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='empty_corridor'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('wheel_tf', default_value='false'),
        OpaqueFunction(function=_gz),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(desc),
                                 launch_arguments={'use_sim_time': 'true'}.items()),
        OpaqueFunction(function=_spawn),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge', output='screen',
             parameters=[{'config_file': os.path.join(share, 'config', 'bridge.yaml'), 'use_sim_time': True}]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='wheel_tf_bridge', output='screen',
             condition=IfCondition(LaunchConfiguration('wheel_tf')),
             arguments=['/odom/wheel_tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
             remappings=[('/odom/wheel_tf', '/tf')],
             parameters=[{'use_sim_time': True}]),
    ])
