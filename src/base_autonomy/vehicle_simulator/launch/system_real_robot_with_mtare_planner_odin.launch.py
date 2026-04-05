import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
  mtare_planner_config = LaunchConfiguration('mtare_planner_config')
  world_name = LaunchConfiguration('world_name')
  sensorOffsetX = LaunchConfiguration('sensorOffsetX')
  sensorOffsetY = LaunchConfiguration('sensorOffsetY')
  cameraOffsetZ = LaunchConfiguration('cameraOffsetZ')
  vehicleX = LaunchConfiguration('vehicleX')
  vehicleY = LaunchConfiguration('vehicleY')
  checkTerrainConn = LaunchConfiguration('checkTerrainConn')
  odin_config_file = LaunchConfiguration('odin_config_file')

  ld = LaunchDescription()
  for name, default in [
      ('mtare_planner_config', 'indoor'),
      ('world_name', 'real_world'),
      ('sensorOffsetX', '0.0'),
      ('sensorOffsetY', '0.0'),
      ('cameraOffsetZ', '0.16'),
      ('vehicleX', '0.0'),
      ('vehicleY', '0.0'),
      ('checkTerrainConn', 'true'),
      ('odin_config_file', os.path.join(
          get_package_share_directory('odin_ros_driver'), 'config', 'control_command.yaml')),
  ]:
    ld.add_action(DeclareLaunchArgument(name, default_value=default, description=''))

  ld.add_action(IncludeLaunchDescription(
    FrontendLaunchDescriptionSource(os.path.join(
      get_package_share_directory('local_planner'), 'launch', 'local_planner.launch')),
    launch_arguments={
      'sensorOffsetX': sensorOffsetX,
      'sensorOffsetY': sensorOffsetY,
      'cameraOffsetZ': cameraOffsetZ,
      'goalX': vehicleX,
      'goalY': vehicleY,
    }.items()))

  ld.add_action(IncludeLaunchDescription(
    FrontendLaunchDescriptionSource(os.path.join(
      get_package_share_directory('terrain_analysis'), 'launch', 'terrain_analysis.launch'))))

  ld.add_action(IncludeLaunchDescription(
    FrontendLaunchDescriptionSource(os.path.join(
      get_package_share_directory('terrain_analysis_ext'), 'launch', 'terrain_analysis_ext.launch')),
    launch_arguments={'checkTerrainConn': checkTerrainConn}.items()))

  ld.add_action(IncludeLaunchDescription(
    FrontendLaunchDescriptionSource(os.path.join(
      get_package_share_directory('sensor_scan_generation'), 'launch', 'sensor_scan_generation.launch'))))

  ld.add_action(IncludeLaunchDescription(
    FrontendLaunchDescriptionSource(os.path.join(
      get_package_share_directory('visualization_tools'), 'launch', 'visualization_tools.launch')),
    launch_arguments={'world_name': world_name}.items()))

  ld.add_action(Node(
    package='joy',
    executable='joy_node',
    name='ps3_joy',
    output='screen',
    parameters=[{'dev': '/dev/input/js0', 'deadzone': 0.12, 'autorepeat_rate': 0.0}]))

  ld.add_action(IncludeLaunchDescription(
    PythonLaunchDescriptionSource([get_package_share_directory('mtare_planner'),
                                   '/explore_world.launch']),
    launch_arguments={'scenario': mtare_planner_config}.items()))

  ld.add_action(IncludeLaunchDescription(
    FrontendLaunchDescriptionSource(os.path.join(
      get_package_share_directory('receive_theta'), 'launch', 'receive_theta.launch'))))

  ld.add_action(Node(
    package='diablo_ctrl',
    executable='diablo_ctrl_node',
    output='screen'
  ))

  ld.add_action(Node(
    package='odin_ros_driver',
    executable='host_sdk_sample',
    name='host_sdk_sample',
    output='screen',
    parameters=[{'config_file': odin_config_file}]
  ))

  ld.add_action(IncludeLaunchDescription(
    PythonLaunchDescriptionSource([get_package_share_directory('odin_autonomy_bridge'),
                                   '/launch/odin_autonomy_bridge.launch.py'])))

  return ld
