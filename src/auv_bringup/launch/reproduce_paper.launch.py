# Launches the full stack AND the scenario runner AND records a rosbag.
# After the run completes (~300 sim-seconds) use plot_from_bag.py to produce
# paper-style figures.
#
# Usage:
#     ros2 launch auv_bringup reproduce_paper.launch.py \
#         scenario:=fig6_abrupt_thrust  bag_out:=/tmp/run1
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    scenario = LaunchConfiguration("scenario")
    bag_out  = LaunchConfiguration("bag_out")

    full_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('auv_bringup'), 'launch', 'full_simulation.launch.py'])),
    )

    # Scenario runner — waits 150 s (sim-time) then injects the configured
    # fault, then terminates the whole graph after 300 s.
    scenario_node = TimerAction(
        period=6.0,   # wait for Gazebo + controller to settle
        actions=[Node(
            package="auv_control",
            executable="scenario_runner.py",
            name="scenario_runner",
            parameters=[{"scenario": scenario, "use_sim_time": True}],
            output="screen",
        )],
    )

    # rosbag2 recording of the signals plot_from_bag.py consumes.
    bag = TimerAction(
        period=4.0,
        actions=[ExecuteProcess(
            cmd=["ros2", "bag", "record",
                 "-o", bag_out,
                 "--use-sim-time",
                 "/auv/virtual_u", "/auv/tau_des",
                 "/auv/tau_actual", "/auv/fault_status",
                 "/auv/odom"],
            output="screen",
        )],
    )

    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value="fig6_abrupt_thrust"),
        DeclareLaunchArgument("bag_out",  default_value="/tmp/auv_ftc_run"),
        full_sim,
        bag,
        scenario_node,
    ])
