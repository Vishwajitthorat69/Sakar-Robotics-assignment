#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='ros2_mobile_manipulator_full', executable='mock_arm_node', output='screen'),
        Node(package='ros2_mobile_manipulator_full', executable='mission_control_node', output='screen')
    ])
