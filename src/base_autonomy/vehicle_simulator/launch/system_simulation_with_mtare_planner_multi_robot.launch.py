import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_test_id(robot_num):
    return f'00{robot_num:02d}'


def _robot_types_arg(robot_types, robot_num):
    robot_types = [item.strip() for item in robot_types.split(',') if item.strip()]
    if not robot_types:
        return ','.join(['wheeled'] * robot_num)
    if len(robot_types) == 1 and robot_num > 1:
        return ','.join(robot_types * robot_num)
    if len(robot_types) != robot_num:
        raise RuntimeError(
            f'robot_types expects 1 or {robot_num} entries, got {len(robot_types)}: {robot_types}'
        )
    return ','.join(robot_types)


def launch_robots(context):
    robot_num = int(LaunchConfiguration('robot_num').perform(context))
    robot_spacing = float(LaunchConfiguration('robot_spacing').perform(context))
    test_id = str(LaunchConfiguration('test_id').perform(context)).strip() or _default_test_id(robot_num)
    robot_types = _robot_types_arg(str(LaunchConfiguration('robot_types').perform(context)), robot_num)
    single_launch = PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('vehicle_simulator'),
            'launch',
            'system_simulation_with_mtare_planner.launch.py',
        )
    )

    actions = []
    for robot_id in range(robot_num):
        vehicle_x = robot_spacing * robot_id
        vehicle_yaw = 0.0 if robot_id == 0 else math.pi
        actions.append(
            IncludeLaunchDescription(
                single_launch,
                launch_arguments={
                    'mtare_planner_config': LaunchConfiguration('mtare_planner_config'),
                    'world_name': LaunchConfiguration('world_name'),
                    'vehicleHeight': LaunchConfiguration('vehicleHeight'),
                    'cameraOffsetZ': LaunchConfiguration('cameraOffsetZ'),
                    'terrainZ': LaunchConfiguration('terrainZ'),
                    'checkTerrainConn': LaunchConfiguration('checkTerrainConn'),
                    'use_boundary': 'true',
                    'robot_id': str(robot_id),
                    'robot_num': str(robot_num),
                    'robot_ns': f'robot_{robot_id}',
                    'ros_tcp_port': str(10000 + robot_id),
                    'coordination': LaunchConfiguration('coordination'),
                    'kAutoStart': LaunchConfiguration('kAutoStart'),
                    'test_id': test_id,
                    'robot_types': robot_types,
                    'vehicleX': str(vehicle_x),
                    'vehicleY': '0.0',
                    'vehicleYaw': str(vehicle_yaw),
                    'launch_visualization': LaunchConfiguration('launch_visualization'),
                    'launch_joy': 'false',
                }.items(),
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mtare_planner_config', default_value='indoor', description=''),
        DeclareLaunchArgument('world_name', default_value='unity', description=''),
        DeclareLaunchArgument('vehicleHeight', default_value='0.75', description=''),
        DeclareLaunchArgument('cameraOffsetZ', default_value='0.041', description=''),
        DeclareLaunchArgument('terrainZ', default_value='0.0', description=''),
        DeclareLaunchArgument('checkTerrainConn', default_value='true', description=''),
        DeclareLaunchArgument('robot_num', default_value='2', description='Number of robots'),
        DeclareLaunchArgument('robot_spacing', default_value='10.0', description='Spacing between robot spawns'),
        DeclareLaunchArgument('coordination', default_value='true', description='Enable coordination'),
        DeclareLaunchArgument('kAutoStart', default_value='true', description='Start exploration automatically'),
        DeclareLaunchArgument('test_id', default_value='', description='MTARE test id'),
        DeclareLaunchArgument(
            'robot_types',
            default_value='',
            description='Comma-separated robot types, defaults to wheeled',
        ),
        DeclareLaunchArgument(
            'launch_visualization',
            default_value='true',
            description='Launch one visualization node per robot',
        ),
        DeclareLaunchArgument(
            'launch_joy',
            default_value='true',
            description='Launch a shared joystick node',
        ),
        DeclareLaunchArgument(
            'unity_path',
            default_value=os.path.expanduser(
                '~/autonomy_stack_diablo_setup/src/base_autonomy/vehicle_simulator/mesh/unity/environment/Model.x86_64'
            ),
            description='Compatibility argument for external scripts that launch Unity separately',
        ),
        DeclareLaunchArgument(
            'use_simulation',
            default_value='true',
            description='Compatibility argument for no-Unity test scripts',
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
        OpaqueFunction(function=launch_robots),
    ])
