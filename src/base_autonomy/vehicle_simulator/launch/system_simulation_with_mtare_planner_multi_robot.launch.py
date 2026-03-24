import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration 

def launch_robot(context, robot_id, x, y, yaw, mtare_planner_config, vehicleHeight, cameraOffsetZ, checkTerrainConn):
    """Launch a single robot with given configuration"""
    scenario_str = str(mtare_planner_config.perform(context))
    vehicle_height_str = str(vehicleHeight.perform(context))
    camera_offset_z_str = str(cameraOffsetZ.perform(context))
    check_terrain_conn_str = str(checkTerrainConn.perform(context))
    ns = f'robot_{robot_id}'
    
    # Each robot gets a unique port for Unity communication
    # Unity instances will be started separately on ports 10000, 10001, etc.
    tcp_port = 10000 + robot_id
    
    # Build namespace-specific topic names
    terrain_map_topic = f'/{ns}/terrain_map'
    terrain_map_ext_topic = f'/{ns}/terrain_map_ext'
    state_estimation_topic = f'/{ns}/state_estimation_at_scan'
    registered_scan_topic = f'/{ns}/registered_scan'
    
    # Build library path
    ortools_lib_path = os.path.join(
        os.environ.get('COLCON_PREFIX_PATH', '/home/yy/autonomy_stack_diablo_setup/install').replace('/install', ''),
        'src', 'mtare_planner', 'tare_planner', 'or-tools', 'lib')
    ld_library_path = os.environ.get('LD_LIBRARY_PATH', '')
    full_lib_path = ld_library_path + ':' + ortools_lib_path if ld_library_path else ortools_lib_path
    
    return GroupAction([
        PushRosNamespace(ns),
        # ROS TCP Endpoint for Unity communication (one per robot, different ports)
        Node(
            package='ros_tcp_endpoint',
            executable='default_server_endpoint',
            name='endpoint',
            parameters=[{
                "ROS_IP": "0.0.0.0",
                "ROS_TCP_PORT": tcp_port,
            }]
        ),
        # Image republisher (namespace specific)
        Node(
            package='vehicle_simulator',
            executable='sim_image_repub',
            name='sim_image_repub',
            parameters=[{
                "camera_in_topic": f"/{ns}/camera/image/compressed",
                "camera_raw_out_topic": f"/{ns}/camera/image",
                "sem_in_topic": f"/{ns}/camera/semantic_image/compressed",
                "sem_raw_out_topic": f"/{ns}/camera/semantic_image",
                "depth_in_topic": f"/{ns}/camera/depth/compressed",
                "depth_raw_out_topic": f"/{ns}/camera/depth",
            }]
        ),
        # Local planner
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(os.path.join(
                get_package_share_directory('local_planner'), 'launch', 'local_planner.launch')
            ),
            launch_arguments=[('cameraOffsetZ', camera_offset_z_str), ('goalX', str(x)), ('goalY', str(y))]
        ),
        # Terrain analysis
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(os.path.join(
                get_package_share_directory('terrain_analysis'), 'launch', 'terrain_analysis.launch')
            ))
        ,
        # Terrain analysis ext
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(os.path.join(
                get_package_share_directory('terrain_analysis_ext'), 'launch', 'terrain_analysis_ext.launch')
            ),
            launch_arguments=[('checkTerrainConn', check_terrain_conn_str)]
        ),
        # Vehicle simulator
        Node(
            package='vehicle_simulator',
            executable='vehicleSimulator',
            name='vehicleSimulator',
            output='screen',
            parameters=[{
                'vehicleHeight': float(vehicle_height_str),
                'vehicleX': float(x),
                'vehicleY': float(y),
                'vehicleZ': 0.0,
                'terrainZ': 0.0,
                'vehicleYaw': float(yaw),
                'terrainVoxelSize': 0.05,
                'groundHeightThre': 0.1,
                'adjustZ': True,
                'terrainRadiusZ': 1.0,
                'minTerrainPointNumZ': 5,
                'smoothRateZ': 0.2,
                'adjustIncl': True,
                'terrainRadiusIncl': 2.0,
                'minTerrainPointNumIncl': 5,
                'smoothRateIncl': 0.2,
                'InclFittingThre': 0.2,
                'maxIncl': 0.5,
                'pause': False,
                'world_name': 'unity',
            }]
        ),
        # Sensor scan generation
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(os.path.join(
                get_package_share_directory('sensor_scan_generation'), 'launch', 'sensor_scan_generation.launch')
            ))
        ,
        # Static TF for vehicle frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_vehicle',
            arguments=['0', '0', '0', '0', '0', '0', 'sensor', 'vehicle']
        ),
        # MTARE planner node
        Node(
            package='mtare_planner',
            executable='tare_planner_node',
            name='tare_planner_node',
            output='screen',
            parameters=[
                os.path.join(get_package_share_directory('mtare_planner'), 'config', scenario_str + '.yaml'),
                {
                    'robot_id': robot_id,
                    'kRobotNum': 2,
                    'kTestID': '0002',
                    'robot_types': ['wheeled', 'wheeled'],
                    'kAutoStart': True,
                    'coordination': True,
                    'sub_terrain_map_topic_': terrain_map_topic,
                    'sub_terrain_map_ext_topic_': terrain_map_ext_topic,
                    'sub_state_estimation_topic_': state_estimation_topic,
                    'sub_registered_scan_topic_': registered_scan_topic,
                }
            ],
            env={
                'LD_LIBRARY_PATH': full_lib_path,
                'HOME': os.environ.get('HOME', '/home/yy')
            }
        ),
    ])

def make_launch_robot_func(robot_id, x, y, yaw, mtare_planner_config, vehicleHeight, cameraOffsetZ, checkTerrainConn):
    def launch_robot_func(context):
        return [launch_robot(context, robot_id, x, y, yaw, mtare_planner_config, vehicleHeight, cameraOffsetZ, checkTerrainConn)]
    return launch_robot_func

def generate_launch_description():
    mtare_planner_config = LaunchConfiguration('mtare_planner_config')
    world_name = LaunchConfiguration('world_name')
    vehicleHeight = LaunchConfiguration('vehicleHeight')
    cameraOffsetZ = LaunchConfiguration('cameraOffsetZ')
    checkTerrainConn = LaunchConfiguration('checkTerrainConn')
    robot_num = LaunchConfiguration('robot_num')
    unity_path = LaunchConfiguration('unity_path')
    
    declare_mtare_planner_config = DeclareLaunchArgument('mtare_planner_config', default_value='indoor', description='')
    declare_world_name = DeclareLaunchArgument('world_name', default_value='unity', description='')
    declare_vehicleHeight = DeclareLaunchArgument('vehicleHeight', default_value='0.75', description='')
    declare_cameraOffsetZ = DeclareLaunchArgument('cameraOffsetZ', default_value='0.041', description='')
    declare_checkTerrainConn = DeclareLaunchArgument('checkTerrainConn', default_value='true', description='')
    declare_robot_num = DeclareLaunchArgument('robot_num', default_value='2', description='Number of robots')
    declare_unity_path = DeclareLaunchArgument(
        'unity_path',
        default_value=os.path.expanduser('~/autonomy_stack_diablo_setup/src/base_autonomy/vehicle_simulator/mesh/unity/environment/Model.x86_64'),
        description='Path to Unity executable'
    )
    
    # Common visualization tools (global, not per-robot)
    start_visualization_tools = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(os.path.join(
            get_package_share_directory('visualization_tools'), 'launch', 'visualization_tools.launch')
        ),
        launch_arguments={'world_name': world_name}.items()
    )

    start_joy = Node(
        package='joy', 
        executable='joy_node',
        name='ps3_joy',
        output='screen',
        parameters=[{'dev': "/dev/input/js0", 'deadzone': 0.12, 'autorepeat_rate': 0.0}]
    )

    # Robot 0 at (0, 0)
    robot0 = OpaqueFunction(
        function=make_launch_robot_func(0, 0.0, 0.0, 0.0, 
                                        mtare_planner_config, vehicleHeight, cameraOffsetZ, checkTerrainConn)
    )

    # Robot 1 at (10, 0)
    robot1 = OpaqueFunction(
        function=make_launch_robot_func(1, 10.0, 0.0, 3.14159, 
                                        mtare_planner_config, vehicleHeight, cameraOffsetZ, checkTerrainConn)
    )

    ld = LaunchDescription()
    
    ld.add_action(declare_mtare_planner_config)
    ld.add_action(declare_world_name)
    ld.add_action(declare_vehicleHeight)
    ld.add_action(declare_cameraOffsetZ)
    ld.add_action(declare_checkTerrainConn)
    ld.add_action(declare_robot_num)
    ld.add_action(declare_unity_path)

    ld.add_action(start_visualization_tools)
    ld.add_action(start_joy)
    ld.add_action(robot0)
    ld.add_action(robot1)

    return ld
