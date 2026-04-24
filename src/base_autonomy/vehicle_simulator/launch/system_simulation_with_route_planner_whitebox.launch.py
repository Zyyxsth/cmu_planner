import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    route_planner_config = LaunchConfiguration("route_planner_config")
    world_name = LaunchConfiguration("world_name")
    vehicleHeight = LaunchConfiguration("vehicleHeight")
    cameraOffsetZ = LaunchConfiguration("cameraOffsetZ")
    vehicleX = LaunchConfiguration("vehicleX")
    vehicleY = LaunchConfiguration("vehicleY")
    terrainZ = LaunchConfiguration("terrainZ")
    vehicleYaw = LaunchConfiguration("vehicleYaw")
    checkTerrainConn = LaunchConfiguration("checkTerrainConn")

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("route_planner_config", default_value="indoor", description=""))
    ld.add_action(DeclareLaunchArgument("world_name", default_value="whitebox_stair_test", description=""))
    ld.add_action(DeclareLaunchArgument("vehicleHeight", default_value="0.75", description=""))
    ld.add_action(DeclareLaunchArgument("cameraOffsetZ", default_value="0.041", description=""))
    ld.add_action(DeclareLaunchArgument("vehicleX", default_value="0.0", description=""))
    ld.add_action(DeclareLaunchArgument("vehicleY", default_value="0.0", description=""))
    ld.add_action(DeclareLaunchArgument("terrainZ", default_value="0.0", description=""))
    ld.add_action(DeclareLaunchArgument("vehicleYaw", default_value="0.0", description=""))
    ld.add_action(DeclareLaunchArgument("checkTerrainConn", default_value="true", description=""))

    ld.add_action(
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                os.path.join(get_package_share_directory("local_planner"), "launch", "local_planner.launch")
            ),
            launch_arguments={"cameraOffsetZ": cameraOffsetZ, "goalX": vehicleX, "goalY": vehicleY}.items(),
        )
    )
    ld.add_action(
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                os.path.join(get_package_share_directory("terrain_analysis"), "launch", "terrain_analysis.launch")
            )
        )
    )
    ld.add_action(
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                os.path.join(get_package_share_directory("terrain_analysis_ext"), "launch", "terrain_analysis_ext.launch")
            ),
            launch_arguments={"checkTerrainConn": checkTerrainConn}.items(),
        )
    )
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("vehicle_simulator"), "launch", "vehicle_simulator.launch")
            ),
            launch_arguments={
                "vehicleHeight": vehicleHeight,
                "vehicleX": vehicleX,
                "vehicleY": vehicleY,
                "terrainZ": terrainZ,
                "vehicleYaw": vehicleYaw,
            }.items(),
        )
    )
    ld.add_action(
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("sensor_scan_generation"),
                    "launch",
                    "sensor_scan_generation.launch",
                )
            )
        )
    )
    ld.add_action(
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                os.path.join(get_package_share_directory("visualization_tools"), "launch", "visualization_tools.launch")
            ),
            launch_arguments={"world_name": world_name}.items(),
        )
    )
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory("far_planner"), "/launch/far_planner.launch"]
            ),
            launch_arguments={"config": route_planner_config}.items(),
        )
    )
    return ld
