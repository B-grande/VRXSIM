#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    vrx_share     = get_package_share_directory('vrx_gz')
    slam_share    = get_package_share_directory('slam_toolbox')
    nav2_share    = get_package_share_directory('nav2_bringup')

    # Default file locations
    default_nav2_params = os.path.join(vrx_share, 'config', 'nav2_params.yaml')
    default_slam_params = os.path.join(vrx_share, 'config', 'slam_params.yaml')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart    = LaunchConfiguration('autostart')
    params_file  = LaunchConfiguration('params_file')
    slam_params  = LaunchConfiguration('slam_params')

    return LaunchDescription([
        # Allow overrides on the command line:
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart',    default_value='true'),
        DeclareLaunchArgument('params_file',  default_value=default_nav2_params),
        DeclareLaunchArgument('slam_params',  default_value=default_slam_params),

        # 1) SLAM Toolbox (online async) from slam_toolbox package
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_share, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file':  slam_params,
            }.items(),
        ),

        # 2) Nav2 navigation from nav2_bringup package
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_share, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart':    autostart,
                'params_file':  params_file,
            }.items(),
        ),
    ])
