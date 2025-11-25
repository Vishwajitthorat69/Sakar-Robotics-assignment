#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess
def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(cmd=['ros2','launch','slam_toolbox','online_async_launch.py'], output='screen')
    ])
