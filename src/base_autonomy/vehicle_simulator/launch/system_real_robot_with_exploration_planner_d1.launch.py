import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
  exploration_planner_config = LaunchConfiguration('exploration_planner_config')
  world_name = LaunchConfiguration('world_name')
  sensorOffsetX = LaunchConfiguration('sensorOffsetX')
  sensorOffsetY = LaunchConfiguration('sensorOffsetY')
  cameraOffsetZ = LaunchConfiguration('cameraOffsetZ')
  vehicleX = LaunchConfiguration('vehicleX')
  vehicleY = LaunchConfiguration('vehicleY')
  checkTerrainConn = LaunchConfiguration('checkTerrainConn')
  d1_odom_topic = LaunchConfiguration('d1_odom_topic')
  d1_imu_topic = LaunchConfiguration('d1_imu_topic')
  d1_cmd_topic = LaunchConfiguration('d1_cmd_topic')

  ld = LaunchDescription()
  for name, default in [
      ('exploration_planner_config', 'indoor'),
      ('world_name', 'real_world'),
      ('sensorOffsetX', '0.0'),
      ('sensorOffsetY', '0.0'),
      ('cameraOffsetZ', '0.16'),
      ('vehicleX', '0.0'),
      ('vehicleY', '0.0'),
      ('checkTerrainConn', 'true'),
      ('d1_odom_topic', '/d1/odom'),
      ('d1_imu_topic', '/d1/imu'),
      ('d1_cmd_topic', '/d1/cmd_vel'),
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
    PythonLaunchDescriptionSource([get_package_share_directory('tare_planner'), '/explore_world.launch']),
    launch_arguments={'scenario': exploration_planner_config}.items()))

  ld.add_action(IncludeLaunchDescription(
    PythonLaunchDescriptionSource([get_package_share_directory('d1_compat_bridge'),
                                   '/launch/d1_compat_bridge.launch.py']),
    launch_arguments={
      'input_odom_topic': d1_odom_topic,
      'input_imu_topic': d1_imu_topic,
      'output_twist_topic': d1_cmd_topic,
    }.items()))

  return ld
