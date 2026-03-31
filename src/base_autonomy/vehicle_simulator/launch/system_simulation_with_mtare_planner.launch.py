import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def _parse_bool(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _topic(robot_ns, leaf):
    return f'/{robot_ns}/{leaf}' if robot_ns else f'/{leaf}'


def _frame(robot_ns, leaf):
    return f'{robot_ns}/{leaf}' if robot_ns else leaf


def _default_robot_ns(robot_id, robot_num, robot_ns):
    robot_ns = robot_ns.strip().strip('/')
    if robot_ns:
        return robot_ns
    if robot_num > 1 or robot_id != 0:
        return f'robot_{robot_id}'
    return ''


def _default_test_id(robot_num):
    return f'00{robot_num:02d}'


def _robot_types_arg(robot_types, robot_num):
    robot_types = [item.strip() for item in robot_types.split(',') if item.strip()]
    if not robot_types:
        return ','.join(['wheeled'] * robot_num)
    return ','.join(robot_types)


def _resolved_int(primary, fallback):
    primary = primary.strip()
    if primary:
        return int(primary)
    return int(fallback)


def _resolved_str(primary, fallback):
    primary = primary.strip()
    if primary:
        return primary
    return fallback


def launch_setup(context):
    mtare_planner_config = str(LaunchConfiguration('mtare_planner_config').perform(context))
    world_name = LaunchConfiguration('world_name')
    vehicle_height = LaunchConfiguration('vehicleHeight')
    camera_offset_z = LaunchConfiguration('cameraOffsetZ')
    vehicle_x = LaunchConfiguration('vehicleX')
    vehicle_y = LaunchConfiguration('vehicleY')
    terrain_z = LaunchConfiguration('terrainZ')
    vehicle_z = LaunchConfiguration('vehicleZ')
    vehicle_yaw = LaunchConfiguration('vehicleYaw')
    check_terrain_conn = LaunchConfiguration('checkTerrainConn')
    use_boundary = LaunchConfiguration('use_boundary')

    robot_id = int(LaunchConfiguration('robot_id').perform(context))
    robot_num = _resolved_int(
        LaunchConfiguration('kRobotNum').perform(context),
        LaunchConfiguration('robot_num').perform(context),
    )
    robot_ns = _default_robot_ns(
        robot_id,
        robot_num,
        str(LaunchConfiguration('robot_ns').perform(context)),
    )
    ros_tcp_port = int(LaunchConfiguration('ros_tcp_port').perform(context))
    coordination = _parse_bool(LaunchConfiguration('coordination').perform(context))
    k_auto_start = _parse_bool(LaunchConfiguration('kAutoStart').perform(context))
    tare_prefix = str(LaunchConfiguration('tare_prefix').perform(context))
    test_id = _resolved_str(
        LaunchConfiguration('kTestID').perform(context),
        str(LaunchConfiguration('test_id').perform(context)).strip() or _default_test_id(robot_num),
    )
    robot_types = _robot_types_arg(str(LaunchConfiguration('robot_types').perform(context)), robot_num)

    terrain_map_topic = _topic(robot_ns, 'terrain_map')
    terrain_map_ext_topic = _topic(robot_ns, 'terrain_map_ext')
    state_estimation_topic = _topic(robot_ns, 'state_estimation')
    state_estimation_at_scan_topic = _topic(robot_ns, 'state_estimation_at_scan')
    registered_scan_topic = _topic(robot_ns, 'registered_scan')
    sensor_scan_topic = _topic(robot_ns, 'sensor_scan')
    colored_registered_scan_topic = _topic(robot_ns, 'registered_scan_colored')
    waypoint_topic = _topic(robot_ns, 'way_point')
    speed_topic = _topic(robot_ns, 'speed')
    check_obstacle_topic = _topic(robot_ns, 'check_obstacle')
    slow_down_topic = _topic(robot_ns, 'slow_down')
    path_topic = _topic(robot_ns, 'path')
    free_paths_topic = _topic(robot_ns, 'free_paths')
    stop_topic = _topic(robot_ns, 'stop')
    cmd_vel_topic = _topic(robot_ns, 'cmd_vel')
    map_clearing_topic = _topic(robot_ns, 'map_clearing')
    cloud_clearing_topic = _topic(robot_ns, 'cloud_clearing')
    runtime_topic = _topic(robot_ns, 'runtime')
    overall_map_topic = _topic(robot_ns, 'overall_map')
    explored_areas_topic = _topic(robot_ns, 'explored_areas')
    trajectory_topic = _topic(robot_ns, 'trajectory')
    explored_volume_topic = _topic(robot_ns, 'explored_volume')
    traveling_distance_topic = _topic(robot_ns, 'traveling_distance')
    time_duration_topic = _topic(robot_ns, 'time_duration')
    model_state_topic = _topic(robot_ns, 'unity_sim/set_model_state')
    camera_prefix = _topic(robot_ns, 'camera')

    sensor_frame = _frame(robot_ns, 'sensor')
    vehicle_frame = _frame(robot_ns, 'vehicle')
    camera_frame = _frame(robot_ns, 'camera')
    sensor_at_scan_frame = _frame(robot_ns, 'sensor_at_scan')

    actions = [
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                os.path.join(get_package_share_directory('local_planner'), 'launch', 'local_planner.launch')
            ),
            launch_arguments={
                'cameraOffsetZ': camera_offset_z,
                'autonomyMode': 'true',
                'goalX': vehicle_x,
                'goalY': vehicle_y,
                'stateEstimationTopic': state_estimation_topic,
                'registeredScanTopic': registered_scan_topic,
                'terrainMapTopic': terrain_map_topic,
                'waypointTopic': waypoint_topic,
                'speedTopic': speed_topic,
                'checkObstacleTopic': check_obstacle_topic,
                'slowDownTopic': slow_down_topic,
                'pathTopic': path_topic,
                'freePathsTopic': free_paths_topic,
                'stopTopic': stop_topic,
                'cmdVelTopic': cmd_vel_topic,
                'mapClearingTopic': map_clearing_topic,
                'cloudClearingTopic': cloud_clearing_topic,
                'sensorFrame': sensor_frame,
                'vehicleFrame': vehicle_frame,
                'cameraFrame': camera_frame,
            }.items(),
        ),
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                os.path.join(get_package_share_directory('terrain_analysis'), 'launch', 'terrain_analysis.launch')
            ),
            launch_arguments={
                'stateEstimationTopic': state_estimation_topic,
                'registeredScanTopic': registered_scan_topic,
                'mapClearingTopic': map_clearing_topic,
                'terrainMapTopic': terrain_map_topic,
            }.items(),
        ),
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                os.path.join(get_package_share_directory('terrain_analysis_ext'), 'launch', 'terrain_analysis_ext.launch')
            ),
            launch_arguments={
                'checkTerrainConn': check_terrain_conn,
                'stateEstimationTopic': state_estimation_topic,
                'registeredScanTopic': registered_scan_topic,
                'cloudClearingTopic': cloud_clearing_topic,
                'terrainMapTopic': terrain_map_topic,
                'terrainMapExtTopic': terrain_map_ext_topic,
            }.items(),
        ),
        Node(
            package='ros_tcp_endpoint',
            executable='default_server_endpoint',
            name='endpoint',
            parameters=[{
                'ROS_IP': '0.0.0.0',
                'ROS_TCP_PORT': ros_tcp_port,
            }],
        ),
        Node(
            package='vehicle_simulator',
            executable='sim_image_repub',
            name='sim_image_repub',
            parameters=[{
                'camera_in_topic': f'{camera_prefix}/image/compressed',
                'camera_raw_out_topic': f'{camera_prefix}/image',
                'sem_in_topic': f'{camera_prefix}/semantic_image/compressed',
                'sem_raw_out_topic': f'{camera_prefix}/semantic_image',
                'depth_in_topic': f'{camera_prefix}/depth/compressed',
                'depth_raw_out_topic': f'{camera_prefix}/depth',
            }],
        ),
        Node(
            package='vehicle_simulator',
            executable='vehicleSimulator',
            name='vehicleSimulator',
            output='screen',
            parameters=[{
                'vehicleHeight': vehicle_height,
                'vehicleX': vehicle_x,
                'vehicleY': vehicle_y,
                'vehicleZ': vehicle_z,
                'terrainZ': terrain_z,
                'vehicleYaw': vehicle_yaw,
                'terrainVoxelSize': 0.05,
                'groundHeightThre': 0.1,
                'adjustZ': True,
                'terrainRadiusZ': 1.0,
                'minTerrainPointNumZ': 5,
                'smoothRateZ': 0.2,
                'adjustIncl': True,
                'terrainRadiusIncl': 2.0,
                'minTerrainPointNumIncl': 200,
                'smoothRateIncl': 0.2,
                'InclFittingThre': 0.2,
                'maxIncl': 30.0,
                'pause': False,
                'world_name': world_name,
                'terrainMapTopic': terrain_map_topic,
                'cmdVelTopic': cmd_vel_topic,
                'stateEstimationTopic': state_estimation_topic,
                'modelStateTopic': model_state_topic,
                'mapFrame': 'map',
                'sensorFrame': sensor_frame,
            }],
        ),
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                os.path.join(get_package_share_directory('sensor_scan_generation'), 'launch', 'sensor_scan_generation.launch')
            ),
            launch_arguments={
                'stateEstimationTopic': state_estimation_topic,
                'registeredScanTopic': registered_scan_topic,
                'stateEstimationAtScanTopic': state_estimation_at_scan_topic,
                'sensorScanTopic': sensor_scan_topic,
                'coloredRegisteredScanTopic': colored_registered_scan_topic,
                'mapFrame': 'map',
                'sensorAtScanFrame': sensor_at_scan_frame,
                'robot_id': str(robot_id),
                'use_robot_color': 'true',
                'publish_colored_registered_scan': 'true',
            }.items(),
        ),
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                os.path.join(get_package_share_directory('visualization_tools'), 'launch', 'visualization_tools.launch')
            ),
            launch_arguments={
                'world_name': world_name,
                'stateEstimationTopic': state_estimation_topic,
                'registeredScanTopic': registered_scan_topic,
                'runtimeTopic': runtime_topic,
                'overallMapTopic': overall_map_topic,
                'exploredAreasTopic': explored_areas_topic,
                'trajectoryTopic': trajectory_topic,
                'exploredVolumeTopic': explored_volume_topic,
                'travelingDistanceTopic': traveling_distance_topic,
                'timeDurationTopic': time_duration_topic,
            }.items(),
            condition=IfCondition(LaunchConfiguration('launch_visualization')),
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='ps3_joy',
            output='screen',
            condition=IfCondition(LaunchConfiguration('launch_joy')),
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.12,
                'autorepeat_rate': 0.0,
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('mtare_tare_planner'), 'explore_namespaced.launch.py')
            ),
            launch_arguments={
                'scenario': mtare_planner_config,
                'robot_ns': '',
                'tare_prefix': tare_prefix,
                'kAutoStart': str(k_auto_start).lower(),
                'robot_id': str(robot_id),
                'kRobotNum': str(robot_num),
                'kTestID': test_id,
                'robot_types': robot_types,
                    'sub_state_estimation_topic_': state_estimation_topic,
                'sub_registered_scan_topic_': registered_scan_topic,
                'sub_terrain_map_topic_': terrain_map_topic,
                'sub_terrain_map_ext_topic_': terrain_map_ext_topic,
                'pub_runtime_topic_': runtime_topic,
                'pub_waypoint_topic_': waypoint_topic,
            }.items(),
        ),
    ]

    if robot_ns:
        return [GroupAction([PushRosNamespace(robot_ns), *actions])]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mtare_planner_config', default_value='indoor', description=''),
        DeclareLaunchArgument('world_name', default_value='unity', description=''),
        DeclareLaunchArgument('vehicleHeight', default_value='0.75', description=''),
        DeclareLaunchArgument('cameraOffsetZ', default_value='0.041', description=''),
        DeclareLaunchArgument('vehicleX', default_value='0.0', description=''),
        DeclareLaunchArgument('vehicleY', default_value='0.0', description=''),
        DeclareLaunchArgument('vehicleZ', default_value='0.0', description=''),
        DeclareLaunchArgument('terrainZ', default_value='0.0', description=''),
        DeclareLaunchArgument('vehicleYaw', default_value='0.0', description=''),
        DeclareLaunchArgument('checkTerrainConn', default_value='true', description=''),
        DeclareLaunchArgument('use_boundary', default_value='true', description='Publish navigation boundary'),
        DeclareLaunchArgument('robot_id', default_value='0', description='Robot ID'),
        DeclareLaunchArgument('robot_num', default_value='1', description='Total number of robots'),
        DeclareLaunchArgument('kRobotNum', default_value='', description='Compatibility alias for robot_num'),
        DeclareLaunchArgument('robot_ns', default_value='', description='Robot namespace'),
        DeclareLaunchArgument('ros_tcp_port', default_value='10000', description='ROS TCP endpoint port'),
        DeclareLaunchArgument('coordination', default_value='false', description='Enable coordination'),
        DeclareLaunchArgument('kAutoStart', default_value='false', description='Start exploration automatically'),
        DeclareLaunchArgument('tare_prefix', default_value='', description='Optional launch prefix for tare_planner_node'),
        DeclareLaunchArgument('launch_visualization', default_value='true', description='Launch visualization tools'),
        DeclareLaunchArgument('launch_joy', default_value='true', description='Launch joystick node'),
        DeclareLaunchArgument('test_id', default_value='', description='MTARE test id'),
        DeclareLaunchArgument('kTestID', default_value='', description='Compatibility alias for test_id'),
        DeclareLaunchArgument(
            'robot_types',
            default_value='',
            description='Comma-separated robot types, defaults to wheeled',
        ),
        OpaqueFunction(function=launch_setup),
    ])
