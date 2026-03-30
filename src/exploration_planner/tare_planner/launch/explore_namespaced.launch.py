import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
import yaml


def _load_params(share_dir, scenario):
    with open(os.path.join(share_dir, 'config', scenario + '.yaml'), 'r', encoding='utf-8') as scenario_file:
        raw = yaml.safe_load(scenario_file) or {}
    if 'tare_planner_node' in raw:
        return raw['tare_planner_node'].get('ros__parameters', {})
    return raw


def launch_tare_node(context):
    share_dir = get_package_share_directory('tare_planner')
    scenario = str(LaunchConfiguration('scenario').perform(context))
    robot_ns = str(LaunchConfiguration('robot_ns').perform(context)).strip().strip('/')
    k_auto_start = str(LaunchConfiguration('kAutoStart').perform(context)).strip().lower() in ('1', 'true', 'yes', 'on')

    scenario_params = _load_params(share_dir, scenario)

    tare_planner_node = Node(
        package='tare_planner',
        executable='tare_planner_node',
        name='tare_planner_node',
        namespace=robot_ns,
        output='screen',
        parameters=[
            scenario_params,
            {
                'kAutoStart': k_auto_start,
                'sub_state_estimation_topic_': LaunchConfiguration('sub_state_estimation_topic_'),
                'sub_registered_scan_topic_': LaunchConfiguration('sub_registered_scan_topic_'),
                'sub_terrain_map_topic_': LaunchConfiguration('sub_terrain_map_topic_'),
                'sub_terrain_map_ext_topic_': LaunchConfiguration('sub_terrain_map_ext_topic_'),
                'pub_runtime_topic_': LaunchConfiguration('pub_runtime_topic_'),
                'pub_waypoint_topic_': LaunchConfiguration('pub_waypoint_topic_'),
            },
        ],
    )
    return [tare_planner_node]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true',
        ),
        DeclareLaunchArgument(
            'scenario',
            default_value='indoor',
            description='Exploration scenario',
        ),
        DeclareLaunchArgument('robot_ns', default_value='', description='Robot namespace'),
        DeclareLaunchArgument('kAutoStart', default_value='false', description='Start exploration automatically'),
        DeclareLaunchArgument(
            'sub_state_estimation_topic_',
            default_value='/state_estimation_at_scan',
            description='Planner state estimation topic',
        ),
        DeclareLaunchArgument(
            'sub_registered_scan_topic_',
            default_value='/registered_scan',
            description='Planner registered scan topic',
        ),
        DeclareLaunchArgument(
            'sub_terrain_map_topic_',
            default_value='/terrain_map',
            description='Planner terrain map topic',
        ),
        DeclareLaunchArgument(
            'sub_terrain_map_ext_topic_',
            default_value='/terrain_map_ext',
            description='Planner extended terrain map topic',
        ),
        DeclareLaunchArgument(
            'pub_runtime_topic_',
            default_value='/runtime',
            description='Planner runtime topic',
        ),
        DeclareLaunchArgument(
            'pub_waypoint_topic_',
            default_value='/way_point',
            description='Planner waypoint topic',
        ),
        SetParameter(name='use_sim_time', value=use_sim_time),
        OpaqueFunction(function=launch_tare_node),
    ])
