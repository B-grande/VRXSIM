#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import vrx_gz.launch
from vrx_gz.model import Model


def launch(context, *args, **kwargs):
    # Retrieve launch arguments
    world            = LaunchConfiguration('world').perform(context)
    sim_mode         = LaunchConfiguration('sim_mode').perform(context)
    bridge_topics    = LaunchConfiguration('bridge_competition_topics').perform(context).lower() == 'true'
    config_file      = LaunchConfiguration('config_file').perform(context)
    robot_name       = LaunchConfiguration('robot').perform(context)
    headless         = LaunchConfiguration('headless').perform(context).lower() == 'true'
    urdf_override    = LaunchConfiguration('urdf').perform(context)
    paused           = LaunchConfiguration('paused').perform(context).lower() == 'true'
    competition_mode = LaunchConfiguration('competition_mode').perform(context).lower() == 'true'
    extra_args       = LaunchConfiguration('extra_gz_args').perform(context)

    launch_processes = []

    # 1) Gazebo simulation
    world_base, _ = os.path.splitext(os.path.basename(world))
    launch_processes.extend(
        vrx_gz.launch.simulation(world_base, headless, paused, extra_args)
    )

    # 2) Spawn the WAM-V (or whatever is in your config_file)
    models = []
    if config_file:
        with open(config_file) as f:
            models = Model.FromConfig(f)
    else:
        m = Model('wamv', 'wam-v', [-532, 162, 0, 0, 0, 1])
        if urdf_override:
            m.set_urdf(urdf_override)
        models.append(m)
    launch_processes.extend(
        vrx_gz.launch.spawn(sim_mode, world_base, models, robot_name)
    )

    # 3) Competition bridges (if enabled)
    if sim_mode in ['bridge', 'full'] and bridge_topics:
        launch_processes.extend(
            vrx_gz.launch.competition_bridges(world_base, competition_mode)
        )

    # 4) Sensor bridges
    if sim_mode in ['bridge', 'full']:
        # IMU & GPS → ROS
        launch_processes.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='imu_gps_bridge',
            output='screen',
            arguments=[
                f'/world/{world_base}/model/wamv/link/wamv/imu_wamv_link/'
                  'sensor/imu_wamv_sensor/imu@ignition.msgs.IMU[sensor_msgs/msg/Imu',
                f'/world/{world_base}/model/wamv/link/wamv/gps_wamv_link/'
                  'sensor/gps_sensor/fix@ignition.msgs.Gps[nav_msgs/msg/NavSatFix'
            ],
        ))
        # Pose → Odometry
        launch_processes.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='pose_to_odom_bridge',
            output='screen',
            arguments=[
                f'/world/{world_base}/model/wamv/pose@ignition.msgs.Pose_V[nav_msgs/msg/Odometry'
            ],
            remappings=[(f'/world/{world_base}/model/wamv/pose', '/wamv/odom')]
        ))
        # NavSat transform
        launch_processes.append(Node(
            package='robot_localization', executable='navsat_transform_node',
            name='navsat_transform', namespace='wamv', output='screen',
            parameters=[PathJoinSubstitution([
                get_package_share_directory('vrx_gz'),
                'config',
                'navsat_transform.yaml'
            ])]
        ))
        # cmd_vel bridge
        launch_processes.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='cmd_vel_bridge',
            output='screen',
            arguments=['/wamv/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist']
        ))
        # downward lidar for SLAM & costmap
        launch_processes.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='laser_down_bridge',
            output='screen',
            arguments=[
                f'/world/{world_base}/model/wamv/sensors/lidars/laser_down/scan'
                  '@ignition.msgs.LaserScan[sensor_msgs/msg/LaserScan'
            ],
        ))

    # 5) EKF fusion
    launch_processes.append(Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node',
        output='screen',
        parameters=[PathJoinSubstitution([
            get_package_share_directory('vrx_gz'),
            'config',
            'ekf.yaml'
        ])]
    ))

    # 6) SLAM Toolbox
    launch_processes.append(Node(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox', output='screen',
        parameters=[
            PathJoinSubstitution([
                get_package_share_directory('vrx_gz'),
                'config',
                'slam_params.yaml'
            ]),
            {'scan_topic': '/wamv/sensors/lidars/laser_down/scan'}
        ],
        remappings=[('scan', '/wamv/sensors/lidars/laser_down/scan')]
    ))

    # 7) Static map→odom transform
    launch_processes.append(Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='map_to_odom', output='screen',
        arguments=['0','0','0','0','0','0','map','odom']
    ))

    # 8) **Static odom→base_link transform**  
    #    (so rviz & Nav2 see your robot in the odom frame)
    launch_processes.append(Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='odom_to_base_link', output='screen',
        arguments=[
            '0','0','0',        # x y z
            '0','0','0','1',    # qx qy qz qw
            'odom',             # parent frame
            'wamv/wamv/base_link'  # child frame
        ]
    ))

    # 9) Map server (subscribe to SLAM map)
    launch_processes.append(Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'mode': 'subscription',
            'subscribe_to_map_topic': True,
            'map_topic': '/map'
        }]
    ))

    # 10) Nav2 bringup (autostart=False → click “Startup” in RViz)
    launch_processes.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'autostart':   'False',
            'params_file': os.path.join(
                get_package_share_directory('vrx_gz'),
                'config',
                'nav2_params.yaml'
            )
        }.items()
    ))

    return launch_processes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world',                     default_value='sydney_regatta'),
        DeclareLaunchArgument('sim_mode',                  default_value='full'),
        DeclareLaunchArgument('bridge_competition_topics', default_value='True'),
        DeclareLaunchArgument('config_file',               default_value=''),
        DeclareLaunchArgument('robot',                     default_value=''),
        DeclareLaunchArgument('headless',                  default_value='False'),
        DeclareLaunchArgument('urdf',                      default_value=''),
        DeclareLaunchArgument('paused',                    default_value='False'),
        DeclareLaunchArgument('competition_mode',          default_value='False'),
        DeclareLaunchArgument('extra_gz_args',             default_value=''),
        OpaqueFunction(function=launch),
    ])
