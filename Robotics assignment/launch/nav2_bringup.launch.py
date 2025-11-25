#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess
def generate_launch_description():
    # Placeholder: use your Nav2 setup/bringup for a full run
    return LaunchDescription([
        ExecuteProcess(cmd=['ros2','launch','nav2_bringup','nav2_bringup_launch.py'], output='screen')
    ])
