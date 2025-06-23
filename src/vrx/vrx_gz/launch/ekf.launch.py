#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import Command  # <--- THIS LINE!
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('vrx_gz')  # or your real package

    # Set these paths as needed
    ekf_config = PathJoinSubstitution([pkg_share, 'config', 'ekf.yaml'])
    urdf_path  = PathJoinSubstitution([pkg_share, 'urdf', 'wamv.urdf'])  # adjust for your file

    return LaunchDescription([
        # 1. Static map->odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        ),

        # 2. robot_state_publisher with xacro
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['xacro', urdf_path])}],
            namespace='wamv'
        ),

        # 3. EKF (robot_localization)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config],
        ),
    ])
