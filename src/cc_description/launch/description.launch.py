"""robot_state_publisher for the courier (use_sim_time on by default)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    xacro_file = PathJoinSubstitution([FindPackageShare('cc_description'), 'urdf', 'courier.urdf.xacro'])
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file,
                 ' wheel_radius_error:=', LaunchConfiguration('wheel_radius_error'),
                 ' wheel_separation_error:=', LaunchConfiguration('wheel_separation_error')]),
        value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('wheel_radius_error', default_value='0.01'),
        DeclareLaunchArgument('wheel_separation_error', default_value='-0.04'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
             output='screen'),
    ])
