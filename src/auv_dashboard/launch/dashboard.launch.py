# Launches rosbridge_websocket and opens the dashboard HTML in a browser.
#
# Usage:
#     ros2 launch auv_dashboard dashboard.launch.py
#
# Requires:
#     sudo apt install -y ros-humble-rosbridge-server
#
# If rosbridge_server is not installed, this launch will print a clear
# error and exit instead of silently failing.
import os
import shutil
import subprocess
import sys
import webbrowser

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            OpaqueFunction, TimerAction)
from launch.substitutions import LaunchConfiguration


def _rosbridge_installed():
    """Return True if the rosbridge_server package is ament-installed."""
    try:
        get_package_share_directory('rosbridge_server')
        return True
    except Exception:
        return False


def open_browser(context, *args, **kwargs):
    dashboard = os.path.join(
        get_package_share_directory('auv_dashboard'),
        'web', 'dashboard.html',
    )
    url = 'file://' + dashboard
    print(f'\n[auv_dashboard] opening dashboard: {url}\n')
    try:
        webbrowser.open(url)
    except Exception as e:
        print(f'[auv_dashboard] could not open browser automatically: {e}')
        print(f'[auv_dashboard] please open: {url}')
    return []


def _setup(context, *args, **kwargs):
    if not _rosbridge_installed():
        print('')
        print('=' * 72)
        print('  [auv_dashboard] ERROR: rosbridge_server is not installed.')
        print('')
        print('  The dashboard needs rosbridge_server to stream live ROS data')
        print('  into the browser. Install it with:')
        print('')
        print('      sudo apt install -y ros-humble-rosbridge-server')
        print('')
        print('  Then re-run:')
        print('      ros2 launch auv_dashboard dashboard.launch.py')
        print('=' * 72)
        print('')
        # Return nothing — the rest of the launch proceeds (or the parent
        # launch continues) without the dashboard.
        return []

    port = LaunchConfiguration('port').perform(context)

    bridge = ExecuteProcess(
        cmd=['ros2', 'run', 'rosbridge_server', 'rosbridge_websocket',
             '--ros-args', '-p', f'port:={port}'],
        output='screen',
    )

    # Give rosbridge a couple of seconds to come up before we open the page.
    browser_opener = TimerAction(
        period=2.5,
        actions=[OpaqueFunction(function=open_browser)],
    )

    return [bridge, browser_opener]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port',         default_value='9090'),
        DeclareLaunchArgument('open_browser', default_value='true'),
        OpaqueFunction(function=_setup),
    ])
