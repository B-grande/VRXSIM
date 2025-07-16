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


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

import vrx_gz.launch
from vrx_gz.model import Model


def launch(context, *args, **kwargs):
    config_file = LaunchConfiguration('config_file').perform(context)
    world_name = LaunchConfiguration('world').perform(context)
    sim_mode = LaunchConfiguration('sim_mode').perform(context)
    bridge_competition_topics = LaunchConfiguration(
        'bridge_competition_topics').perform(context).lower() == 'true'
    robot = LaunchConfiguration('robot').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    robot_urdf = LaunchConfiguration('urdf').perform(context)
    gz_paused = LaunchConfiguration('paused').perform(context).lower() == 'true'
    competition_mode = LaunchConfiguration('competition_mode').perform(context).lower() == 'true'
    extra_gz_args = LaunchConfiguration('extra_gz_args').perform(context)
    robot_name = LaunchConfiguration('name').perform(context)
    model_type = LaunchConfiguration('model').perform(context)
    enable_localization = LaunchConfiguration('enable_localization').perform(context).lower() == 'true'

    launch_processes = []

    models = []
    if config_file and config_file != '':
        with open(config_file, 'r') as stream:
            models = Model.FromConfig(stream)
    else:
      m = Model(robot_name, model_type, [-532, 162, 0, 0, 0, 1])
      if robot_urdf and robot_urdf != '':
          m.set_urdf(robot_urdf)
      models.append(m)

    world_name, ext = os.path.splitext(world_name)
    launch_processes.extend(vrx_gz.launch.simulation(world_name, headless, 
                                                     gz_paused, extra_gz_args))
    world_name_base = os.path.basename(world_name)
    launch_processes.extend(vrx_gz.launch.spawn(sim_mode, world_name_base, models, robot))

    if (sim_mode == 'bridge' or sim_mode == 'full') and bridge_competition_topics:
        launch_processes.extend(vrx_gz.launch.competition_bridges(world_name_base, competition_mode))

    # Add robot localization nodes if enabled
    if enable_localization:
        try:
            vrx_gz_dir = get_package_share_directory('vrx_gz')
            
            # Local EKF node (odom frame)
            ekf_odom_node = Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_local',
                output='screen',
                parameters=[
                    PathJoinSubstitution([
                        vrx_gz_dir,
                        'config',
                        'localization.yaml'
                    ]),
                    {'use_sim_time': True}
                ],
                remappings=[
                    ('/odometry/filtered', '/odometry/local')
                ]
            )

            # NavSat Transform node
            navsat_node = Node(
                package='robot_localization',
                executable='navsat_transform_node',
                name='navsat_transform',
                output='screen',
                parameters=[
                    PathJoinSubstitution([
                        vrx_gz_dir,
                        'config',
                        'localization.yaml'
                    ]),
                    {'use_sim_time': True}
                ],
                remappings=[
                    ('imu/data', '/wamv/sensors/imu/imu/data'),
                    ('gps/fix', '/wamv/sensors/gps/gps/fix'),
                    ('odometry/filtered', '/odometry/local')
                ]
            )
            ekf_map_node = Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_global',
                output='screen',
                parameters=[
                    PathJoinSubstitution([
                        vrx_gz_dir,
                        'config',
                        'localization.yaml'
                    ]),
                    {'use_sim_time': True}
                ],
                remappings=[
                    ('/odometry/filtered', '/odometry/global')
                ]
            )

            launch_processes.extend([
                ekf_odom_node,
                ekf_map_node,
                navsat_node,
            ])

           
            # Add thruster converter for WAM-V navigation
            thruster_converter_node = ExecuteProcess(
                cmd=[   
                    'python3', 
                    '/home/ros2404/ros2_ws/src/vrx/vrx_gz/scripts/cmd_vel_to_thrusters.py'
                ],
                name='cmd_vel_to_thrusters',
                output='screen'
            )
            
            launch_processes.append(thruster_converter_node)

            # Add waypoint bridge node for Vizanti/Nav2 integration
            waypoint_bridge_node = ExecuteProcess(
                cmd=[
                    'python3',
                    '/home/ros2404/ros2_ws/src/vrx/vrx_gz/scripts/waypoint_bridge.py'
                ],
                name='waypoint_bridge',
                output='screen'
            )
            
            launch_processes.append(waypoint_bridge_node)

            # Add RViz for visualization
            rviz_node = Node(
                package='rviz2',
                namespace='',
                executable='rviz2',
                name='rviz2',
                arguments=['-d' + os.path.join(get_package_share_directory('vrx_gazebo'), 'config', 'rviz_vrx_rsp.rviz')],
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
            
            launch_processes.append(rviz_node)

           
            
        except Exception as e:
            print(f"Warning: Could not add localization nodes: {e}")

    return launch_processes


def generate_launch_description():
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'world',
            default_value='sydney_regatta',
            description='Name of world'),
        DeclareLaunchArgument(
            'sim_mode',
            default_value='full',
            description='Simulation mode: "full", "sim", "bridge".'
                        'full: spawns robot and launch ros_gz bridges, '
                        'sim: spawns robot only, '
                        'bridge: launch ros_gz bridges only.'),
        DeclareLaunchArgument(
            'bridge_competition_topics',
            default_value='True',
            description='True to bridge competition topics, False to disable bridge.'),
        DeclareLaunchArgument(
            'config_file',
            default_value='',
            description='YAML configuration file to spawn'),
        DeclareLaunchArgument(
            'robot',
            default_value='',
            description='Name of robot to spawn if specified. '
                        'This must match one of the robots in the config_file'),
        DeclareLaunchArgument(
            'headless',
            default_value='False',
            description='True to run simulation headless (no GUI). '),
        DeclareLaunchArgument(
            'urdf',
            default_value='',
            description='URDF file of the wam-v model. '),
        DeclareLaunchArgument(
            'paused',
            default_value='False',
            description='True to start the simulation paused. '),
        DeclareLaunchArgument(
            'competition_mode',
            default_value='False',
            description='True to disable debug topics. '),
        DeclareLaunchArgument(
            'extra_gz_args',
            default_value='',
            description='Additional arguments to be passed to gz sim. '),
        DeclareLaunchArgument(
            'name',
            default_value='wamv',
            description='Name of robot to spawn'),
        DeclareLaunchArgument(
            'model',
            default_value='wam-v',
            description='SDF model to spawn'),
        DeclareLaunchArgument(
            'enable_localization',
            default_value='False',
            description='True to enable robot localization nodes (EKF + NavSat)'),
        OpaqueFunction(function=launch),
    ])
