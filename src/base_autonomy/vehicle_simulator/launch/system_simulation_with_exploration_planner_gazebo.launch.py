import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def _load_route_gazebo_helpers():
    helper_path = os.path.join(
        os.path.dirname(__file__), "system_simulation_with_route_planner_gazebo.launch.py"
    )
    spec = importlib.util.spec_from_file_location("route_gazebo_launch_helpers", helper_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load Gazebo helper launch file: {helper_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_ROUTE_GAZEBO = _load_route_gazebo_helpers()


def generate_launch_description():
    exploration_planner_config = LaunchConfiguration("exploration_planner_config")
    world_name = LaunchConfiguration("world_name")
    scene_map_path = LaunchConfiguration("scene_map_path")
    boundary_file = LaunchConfiguration("boundary_file")
    vehicle_height = LaunchConfiguration("vehicleHeight")
    camera_offset_z = LaunchConfiguration("cameraOffsetZ")
    vehicle_x = LaunchConfiguration("vehicleX")
    vehicle_y = LaunchConfiguration("vehicleY")
    terrain_z = LaunchConfiguration("terrainZ")
    vehicle_yaw = LaunchConfiguration("vehicleYaw")
    check_terrain_conn = LaunchConfiguration("checkTerrainConn")
    vehicle_terrain_map_topic = LaunchConfiguration("vehicleTerrainMapTopic")

    start_local_planner = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(get_package_share_directory("local_planner"), "launch", "local_planner.launch")
        ),
        launch_arguments={
            "cameraOffsetZ": camera_offset_z,
            "goalX": vehicle_x,
            "goalY": vehicle_y,
        }.items(),
    )

    start_terrain_analysis = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(get_package_share_directory("terrain_analysis"), "launch", "terrain_analysis.launch")
        )
    )

    start_terrain_analysis_ext = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(get_package_share_directory("terrain_analysis_ext"), "launch", "terrain_analysis_ext.launch")
        ),
        launch_arguments={"checkTerrainConn": check_terrain_conn}.items(),
    )

    start_vehicle_simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("vehicle_simulator"), "launch", "vehicle_simulator.launch")
        ),
        launch_arguments={
            "useSimTime": "true",
            "vehicleHeight": vehicle_height,
            "vehicleX": vehicle_x,
            "vehicleY": vehicle_y,
            "terrainZ": terrain_z,
            "vehicleYaw": vehicle_yaw,
            "terrainMapTopic": vehicle_terrain_map_topic,
        }.items(),
    )

    start_registered_scan = Node(
        package="vehicle_simulator",
        executable="registeredScanFromOdom",
        output="screen",
        parameters=[{
            "point_cloud_topic": "/lidar/points",
            "odom_topic": "/state_estimation",
            "registered_scan_topic": "/registered_scan",
            "state_estimation_at_scan_topic": "/state_estimation_at_scan",
            "output_frame": "map",
            "sensor_frame": "sensor_at_scan",
            "publish_state_estimation_at_scan": True,
            "publish_scan_tf": False,
            "sensor_offset_x": 0.0,
            "sensor_offset_y": 0.0,
            "sensor_offset_z": 0.0,
        }],
    )

    start_sensor_scan_generation = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(get_package_share_directory("sensor_scan_generation"), "launch", "sensor_scan_generation.launch")
        )
    )

    start_visualization_tools = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(get_package_share_directory("visualization_tools"), "launch", "visualization_tools.launch")
        ),
        launch_arguments={"world_name": world_name, "mapFile": scene_map_path}.items(),
    )

    start_joy = Node(
        package="joy",
        executable="joy_node",
        name="ps3_joy",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_joy")),
        parameters=[{"dev": "/dev/input/js0", "deadzone": 0.12, "autorepeat_rate": 0.0}],
    )

    start_exploration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("tare_planner"), "explore_world.launch")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "scenario": exploration_planner_config,
            "boundary_file": boundary_file,
        }.items(),
    )

    start_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            ["/world/", world_name, "/set_pose@ros_gz_interfaces/srv/SetEntityPose"],
        ],
    )

    start_pose_adapter = Node(
        package="vehicle_simulator",
        executable="poseStampedToGazeboSetPose",
        output="screen",
        parameters=[{
            "input_topic": "/unity_sim/set_model_state",
            "service_name": ["/world/", world_name, "/set_pose"],
            "model_name": "stairbot",
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("exploration_planner_config", default_value="indoor", description=""),
        DeclareLaunchArgument("world_name", default_value="whitebox_stair_test", description=""),
        DeclareLaunchArgument("vehicleHeight", default_value="0.75", description=""),
        DeclareLaunchArgument("cameraOffsetZ", default_value="0.041", description=""),
        DeclareLaunchArgument("vehicleX", default_value="0.0", description=""),
        DeclareLaunchArgument("vehicleY", default_value="0.0", description=""),
        DeclareLaunchArgument("terrainZ", default_value="0.0", description=""),
        DeclareLaunchArgument("vehicleYaw", default_value="0.0", description=""),
        DeclareLaunchArgument("checkTerrainConn", default_value="true", description=""),
        DeclareLaunchArgument("vehicleTerrainMapTopic", default_value="/terrain_map", description=""),
        DeclareLaunchArgument("gazebo_gui", default_value="true", description=""),
        DeclareLaunchArgument("launch_joy", default_value="true", description=""),
        DeclareLaunchArgument("launch_tare", default_value="true", description=""),
        DeclareLaunchArgument(
            "scene_mesh_path",
            default_value="src/base_autonomy/vehicle_simulator/mesh/whitebox_stair_test/whitebox_stair_test.obj",
            description="Absolute or cwd-relative OBJ mesh path for the Gazebo whitebox scene.",
        ),
        DeclareLaunchArgument(
            "scene_map_path",
            default_value="src/base_autonomy/vehicle_simulator/mesh/whitebox_stair_test/map.ply",
            description="Absolute or cwd-relative PLY map path for RViz /overall_map visualization.",
        ),
        DeclareLaunchArgument(
            "boundary_file",
            default_value="src/exploration_planner/tare_planner/data/whitebox_36m_boundary.ply",
            description="Exploration boundary PLY file.",
        ),
        SetParameter(name="use_sim_time", value=True),
        OpaqueFunction(function=_ROUTE_GAZEBO._write_world_file),
        start_bridge,
        start_pose_adapter,
        start_local_planner,
        start_terrain_analysis,
        start_terrain_analysis_ext,
        start_vehicle_simulator,
        start_registered_scan,
        start_sensor_scan_generation,
        start_visualization_tools,
        start_joy,
        TimerAction(
            period=5.0,
            actions=[start_exploration],
            condition=IfCondition(LaunchConfiguration("launch_tare")),
        ),
    ])
