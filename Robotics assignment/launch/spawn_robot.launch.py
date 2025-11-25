#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import xacro, pathlib, sys

def generate_launch_description():
    arm_choice = LaunchConfiguration('arm', default='A')
    return LaunchDescription([
        DeclareLaunchArgument('arm', default_value='A', description='Choose A (3-DOF) or B (Doosan mount)'),
        OpaqueFunction(function=lambda context, *args, **kwargs: spawn(context))
    ])

def spawn(context):
    pkg = get_package_share_directory('ros2_mobile_manipulator_full')
    arm = LaunchConfiguration('arm').perform(context)
    if arm == 'B':
        urdf_path = os.path.join(pkg, 'urdf', 'tb3_with_doosan.xacro')
    else:
        urdf_path = os.path.join(pkg, 'urdf', 'tb3_with_arm_3dof.xacro')

    # convert xacro to urdf
    doc = xacro.process_file(urdf_path)
    robot_xml = doc.toxml()
    urdf_temp = '/tmp/robot_for_spawn.urdf'
    with open(urdf_temp, 'w') as f:
        f.write(robot_xml)
    # spawn using spawn_entity.py
    from launch.actions import ExecuteProcess
    return [ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'], output='screen'),
            ExecuteProcess(cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py', '-file', urdf_temp, '-entity', 'mobile_manipulator'], output='screen')]
