#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter, LoadComposableNodes
from launch_ros.descriptions import ComposableNode, ParameterFile
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml

import vrx_gz.launch
from vrx_gz.model import Model


def launch(context, *args, **kwargs):
    # VRX launch args (performed)
    world = LaunchConfiguration('world').perform(context)
    sim_mode = LaunchConfiguration('sim_mode').perform(context)
    bridge_topics = LaunchConfiguration('bridge_competition_topics').perform(context).lower() == 'true'
    config_file = LaunchConfiguration('config_file').perform(context)
    robot_name = LaunchConfiguration('robot').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    urdf_override = LaunchConfiguration('urdf').perform(context)
    paused = LaunchConfiguration('paused').perform(context).lower() == 'true'
    competition_mode = LaunchConfiguration('competition_mode').perform(context).lower() == 'true'
    extra_args = LaunchConfiguration('extra_gz_args').perform(context)

    # Nav2 launch args (substitutions)
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    launch_processes = []

    # 1) Gazebo simulation
    world_base = os.path.splitext(os.path.basename(world))[0]
    launch_processes.extend(vrx_gz.launch.simulation(world_base, headless, paused, extra_args))

    # 2) Spawn robot model
    models = []
    if config_file:
        with open(config_file, 'r') as f:
            models = Model.FromConfig(f)
    else:
        m = Model('wamv', 'wam-v', [-532, 162, 0, 0, 0, 1])
        if urdf_override:
            m.set_urdf(urdf_override)
        models.append(m)
    launch_processes.extend(vrx_gz.launch.spawn(sim_mode, world_base, models, robot_name))

    # 3) Competition-topic bridges
    if sim_mode in ['bridge', 'full'] and bridge_topics:
        launch_processes.extend(vrx_gz.launch.competition_bridges(world_base, competition_mode))

    # 4) Sensor bridges
    if sim_mode in ['bridge', 'full']:
        launch_processes.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='imu_gps_bridge', output='screen',
            arguments=[
                f'/world/{world_base}/model/wamv/link/wamv/imu_wamv_link/sensor/imu_wamv_sensor/imu@ignition.msgs.IMU[sensor_msgs/msg/Imu',
                f'/world/{world_base}/model/wamv/link/wamv/gps_wamv_link/sensor/gps_sensor/fix@ignition.msgs.Gps[nav_msgs/msg/NavSatFix'
            ],
        ))
        launch_processes.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='pose_to_odom_bridge', output='screen',
            arguments=[f'/world/{world_base}/model/wamv/pose@ignition.msgs.Pose_V[nav_msgs/msg/Odometry'],
            remappings=[(f'/world/{world_base}/model/wamv/pose','/wamv/odom')]
        ))
        launch_processes.append(Node(
            package='robot_localization', executable='navsat_transform_node', name='navsat_transform', namespace='wamv', output='screen',
            parameters=[PathJoinSubstitution([get_package_share_directory('vrx_gz'),'config','navsat_transform.yaml'])]
        ))
        launch_processes.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='cmd_vel_bridge', output='screen',
            arguments=['/wamv/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist']
        ))
        launch_processes.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='laser_down_bridge', output='screen',
            arguments=[f'/world/{world_base}/model/wamv/sensors/lidars/laser_down/scan@ignition.msgs.LaserScan[sensor_msgs/msg/LaserScan']
        ))

    # 5) EKF sensor fusion
    launch_processes.append(Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
        parameters=[PathJoinSubstitution([get_package_share_directory('vrx_gz'),'config','ekf.yaml'])]
    ))

    # 6) SLAM Toolbox
    launch_processes.append(Node(
        package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox', output='screen',
        parameters=[PathJoinSubstitution([get_package_share_directory('vrx_gz'),'config','slam_params.yaml']),{'scan_topic':'/wamv/sensors/lidars/laser_down/scan'}],
        remappings=[('scan','/wamv/sensors/lidars/laser_down/scan')]
    ))

    # 7) Static TF map->odom
    launch_processes.append(Node(
        package='tf2_ros', executable='static_transform_publisher', name='map_to_odom', output='screen',
        arguments=['0','0','0','0','0','0','map','odom']
    ))

    # 8) Inline Nav2 bringup
    nav2_param_rewrites = {'autostart': autostart}
    configured_nav2 = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=nav2_param_rewrites,
            convert_types=True,
        ), allow_substs=True
    )
    nav2_remap = [('/tf','tf'),('/tf_static','tf_static')]

    nav2_list = [
        ('nav2_map_server', 'map_server', []),
        ('nav2_controller', 'controller_server', [('cmd_vel', 'cmd_vel_nav')]),
        ('nav2_smoother', 'smoother_server', []),
        ('nav2_planner', 'planner_server', []),
        ('nav2_behaviors', 'behavior_server', []),
        ('nav2_bt_navigator', 'bt_navigator', []),
        ('nav2_waypoint_follower', 'waypoint_follower', []),
        ('nav2_velocity_smoother', 'velocity_smoother', [('cmd_vel', 'cmd_vel_nav')]),
    ]
    for pkg, exe, remap in nav2_list:
        launch_processes.append(Node(
            package=pkg,
            executable=exe,
            name=exe,
            output='screen',
            respawn=use_respawn,
            parameters=[configured_nav2],
            remappings=nav2_remap + remap,
            arguments=['--ros-args', '--log-level', log_level]
        ))

    # Lifecycle manager
    node_names = [entry[1] for entry in nav2_list]
    launch_processes.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
        output='screen', respawn=use_respawn,
        parameters=[configured_nav2, {'node_names': node_names}],
        arguments=['--ros-args', '--log-level', log_level]
    ))

    # 9) RViz with model loaded
    launch_processes.append(Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d' + os.path.join(get_package_share_directory('vrx_gazebo'), 'config', 'rviz_vrx_rsp.rviz')]
    ))

    return launch_processes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='sydney_regatta'),
        DeclareLaunchArgument('sim_mode', default_value='full'),
        DeclareLaunchArgument('bridge_competition_topics', default_value='True'),
        DeclareLaunchArgument('config_file', default_value=''),
        DeclareLaunchArgument('robot', default_value=''),
        DeclareLaunchArgument('headless', default_value='False'),
        DeclareLaunchArgument('urdf', default_value=''),
        DeclareLaunchArgument('paused', default_value='False'),
        DeclareLaunchArgument('competition_mode', default_value='False'),
        DeclareLaunchArgument('extra_gz_args', default_value=''),
        # Nav2 args
        DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time'),
        DeclareLaunchArgument('autostart', default_value='true', description='Autostart Nav2 lifecycle nodes'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(get_package_share_directory('vrx_gz'), 'config', 'nav2_params.yaml'), description='Nav2 params file'),
        DeclareLaunchArgument('use_composition', default_value='false', description='Enable composable nodes'),
        DeclareLaunchArgument('container_name', default_value='nav2_container', description='Composition container name'),
        DeclareLaunchArgument('use_respawn', default_value='false', description='Respawn nodes on crash'),
        DeclareLaunchArgument('log_level', default_value='info', description='Logging level'),
        OpaqueFunction(function=launch),
    ])
