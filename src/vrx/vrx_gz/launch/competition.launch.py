# competition.launch.py
# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import vrx_gz.launch
from vrx_gz.model import Model


def launch(context, *args, **kwargs):
    # Retrieve launch arguments
    pkg_share = get_package_share_directory('vrx_gz')
    config_dir = os.path.join(pkg_share, 'config')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Common args
    config_file = LaunchConfiguration('config_file').perform(context)
    world_name = LaunchConfiguration('world').perform(context)
    sim_mode = LaunchConfiguration('sim_mode').perform(context)
    bridge_topics = LaunchConfiguration('bridge_competition_topics').perform(context).lower() == 'true'
    robot = LaunchConfiguration('robot').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    robot_urdf = LaunchConfiguration('urdf').perform(context)
    gz_paused = LaunchConfiguration('paused').perform(context).lower() == 'true'
    competition_mode = LaunchConfiguration('competition_mode').perform(context).lower() == 'true'
    extra_gz_args = LaunchConfiguration('extra_gz_args').perform(context)

    launch_processes = []

    # --- Gazebo sim + spawn ---
    models = []
    if config_file:
        with open(config_file, 'r') as f:
            models = Model.FromConfig(f)
    else:
        m = Model('wamv', 'wam-v', [-532, 162, 0, 0, 0, 1])
        if robot_urdf:
            m.set_urdf(robot_urdf)
        models.append(m)

    world_base, _ = os.path.splitext(world_name)
    launch_processes.extend(vrx_gz.launch.simulation(world_base, headless, gz_paused, extra_gz_args))
    launch_processes.extend(vrx_gz.launch.spawn(sim_mode, world_base, models, robot))
    if sim_mode in ['bridge', 'full'] and bridge_topics:
        launch_processes.extend(vrx_gz.launch.competition_bridges(world_base, competition_mode))

    # --- Robot state estimation ---
    launch_processes.append(
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
             parameters=[os.path.join(config_dir, 'ekf.yaml'), {'use_sim_time': True}])
    )
    launch_processes.append(
        Node(package='robot_localization', executable='navsat_transform_node', name='navsat_transform_node', output='screen',
             parameters=[os.path.join(config_dir, 'navsat_transform.yaml'), {'use_sim_time': True}])
    )

    

    

    # --- Nav2 bringup (delayed) ---
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': 'True',
            'params_file': os.path.join(config_dir, 'nav2_params.yaml'),
            'autostart': 'true',
        }.items()
    )
    launch_processes.append(TimerAction(period=3.0, actions=[bringup]))

    # --- RViz2 ---
    launch_processes.append(
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', os.path.join(get_package_share_directory('vrx_gazebo'), 'config', 'rviz_vrx_rsp.rviz')])
    )

    return launch_processes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='sydney_regatta', description='Name of world'),
        DeclareLaunchArgument('sim_mode', default_value='full', description='Simulation mode: sim, bridge, full'),
        DeclareLaunchArgument('bridge_competition_topics', default_value='True', description='Bridge topics'),
        DeclareLaunchArgument('config_file', default_value='', description='YAML file for models'),
        DeclareLaunchArgument('robot', default_value='', description='Name of robot'),
        DeclareLaunchArgument('headless', default_value='False', description='Run Gazebo headless'),
        DeclareLaunchArgument('urdf', default_value='', description='Override URDF'),
        DeclareLaunchArgument('paused', default_value='False', description='Start paused'),
        DeclareLaunchArgument('competition_mode', default_value='False', description='Competition mode'),
        DeclareLaunchArgument('extra_gz_args', default_value='', description='Extra Gazebo args'),
        DeclareLaunchArgument('use_sim_time', default_value='True', description='Use simulation clock'),
        DeclareLaunchArgument('autostart', default_value='true', description='Nav2 autostart'),
        OpaqueFunction(function=launch),
    ])
