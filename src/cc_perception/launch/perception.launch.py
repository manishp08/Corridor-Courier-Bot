"""ros2 launch cc_perception perception.launch.py backend:=oracle world:=walking_actors"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    manifest = PathJoinSubstitution([FindPackageShare('cc_gazebo'), 'worlds', [LaunchConfiguration('world'), '.json']])
    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='oracle'),
        DeclareLaunchArgument('world', default_value='empty_corridor'),
        DeclareLaunchArgument('model_path', default_value=''),
        DeclareLaunchArgument('seed', default_value='0'),
        Node(package='cc_perception', executable='detector_node', name='cc_detector', output='screen',
             parameters=[{'use_sim_time': True,
                          'backend': LaunchConfiguration('backend'),
                          'model_path': LaunchConfiguration('model_path'),
                          'manifest': manifest,
                          'seed': LaunchConfiguration('seed')}]),
    ])
