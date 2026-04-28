# One-shot: Gazebo + controller + dashboard.
#
# Usage:
#     ros2 launch auv_bringup full_simulation_with_dashboard.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    full_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('auv_bringup'), 'launch', 'full_simulation.launch.py'])),
    )

    dashboard = TimerAction(
        period=4.0,   # let Gazebo + controller come up first
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('auv_dashboard'), 'launch', 'dashboard.launch.py'])),
        )],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        full_sim,
        dashboard,
    ])
