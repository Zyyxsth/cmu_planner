import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
  robot_namespace_default = os.environ.get('ROBOT_NS', 'd15020108').strip('/')
  topic_prefix = f'/{robot_namespace_default}' if robot_namespace_default else ''

  route_planner_config = LaunchConfiguration('route_planner_config')
  world_name = LaunchConfiguration('world_name')
  sensorOffsetX = LaunchConfiguration('sensorOffsetX')
  sensorOffsetY = LaunchConfiguration('sensorOffsetY')
  cameraOffsetZ = LaunchConfiguration('cameraOffsetZ')
  vehicleX = LaunchConfiguration('vehicleX')
  vehicleY = LaunchConfiguration('vehicleY')
  checkTerrainConn = LaunchConfiguration('checkTerrainConn')
  d1_robot = LaunchConfiguration('d1_robot')
  d1_controller_name = LaunchConfiguration('d1_controller_name')
  d1_can_interface = LaunchConfiguration('d1_can_interface')
  start_d1_traditional_hw = LaunchConfiguration('start_d1_traditional_hw')
  start_odin_driver = LaunchConfiguration('start_odin_driver')
  start_receive_theta = LaunchConfiguration('start_receive_theta')
  showImage = LaunchConfiguration('showImage')
  odin_config_file = LaunchConfiguration('odin_config_file')
  d1_joint_states_topic = LaunchConfiguration('d1_joint_states_topic')
  d1_imu_topic = LaunchConfiguration('d1_imu_topic')
  d1_fsm_topic = LaunchConfiguration('d1_fsm_topic')
  d1_motors_status_topic = LaunchConfiguration('d1_motors_status_topic')
  d1_cmd_topic = LaunchConfiguration('d1_cmd_topic')
  d1_key_topic = LaunchConfiguration('d1_key_topic')
  d1_standup_key = LaunchConfiguration('d1_standup_key')
  d1_standdown_key = LaunchConfiguration('d1_standdown_key')
  d1_ros_domain_id = LaunchConfiguration('d1_ros_domain_id')
  d1_ros_localhost_only = LaunchConfiguration('d1_ros_localhost_only')
  autonomy_mode = LaunchConfiguration('autonomy_mode')
  autonomy_speed = LaunchConfiguration('autonomy_speed')
  startup_delay_sec = LaunchConfiguration('startup_delay_sec')
  odin_base_frame = LaunchConfiguration('odin_base_frame')
  planner_sensor_frame = LaunchConfiguration('planner_sensor_frame')
  odin_cloud_topic = LaunchConfiguration('odin_cloud_topic')

  ld = LaunchDescription()
  for name, default in [
      ('route_planner_config', 'indoor'),
      ('world_name', 'real_world'),
      ('sensorOffsetX', '0.0'),
      ('sensorOffsetY', '0.0'),
      ('cameraOffsetZ', '0.16'),
      ('vehicleX', '0.0'),
      ('vehicleY', '0.0'),
      ('checkTerrainConn', 'true'),
      ('d1_robot', 'd1'),
      ('d1_controller_name', 'd1_rl_controller'),
      ('d1_can_interface', 'can0'),
      ('start_d1_traditional_hw', 'false'),
      ('start_odin_driver', 'true'),
      ('start_receive_theta', 'false'),
      ('showImage', 'false'),
      ('odin_config_file', os.path.join(
          get_package_share_directory('odin_ros_driver'), 'config', 'control_command.yaml')),
      ('d1_joint_states_topic', f'{topic_prefix}/joint_states'),
      ('d1_imu_topic', f'{topic_prefix}/imu_sensor_broadcaster/imu'),
      ('d1_fsm_topic', f'{topic_prefix}/rl_controller/fsm'),
      ('d1_motors_status_topic', f'{topic_prefix}/system_status_broadcaster/motors_status'),
      ('d1_cmd_topic', f'{topic_prefix}/command/cmd_twist'),
      ('d1_key_topic', f'{topic_prefix}/command/cmd_key'),
      ('d1_standup_key', 'transform_up'),
      ('d1_standdown_key', 'transform_down'),
      ('d1_ros_domain_id', '42'),
      ('d1_ros_localhost_only', '1'),
      ('autonomy_mode', 'false'),
      ('autonomy_speed', '0.3'),
      ('startup_delay_sec', '8.0'),
      ('odin_base_frame', 'odin1_base_link'),
      ('planner_sensor_frame', 'sensor'),
      ('odin_cloud_topic', '/odin1/cloud_slam'),
  ]:
    ld.add_action(DeclareLaunchArgument(name, default_value=default, description=''))

  ld.add_action(SetEnvironmentVariable('ROS_DOMAIN_ID', d1_ros_domain_id))
  ld.add_action(SetEnvironmentVariable('ROS_LOCALHOST_ONLY', d1_ros_localhost_only))

  ld.add_action(IncludeLaunchDescription(
    PythonLaunchDescriptionSource([get_package_share_directory('d1_compat_bridge'),
                                   '/launch/d1_traditional_hw.launch.py']),
    launch_arguments={
      'robot': d1_robot,
      'controller_name': d1_controller_name,
      'can_interface': d1_can_interface,
    }.items(),
    condition=IfCondition(start_d1_traditional_hw)))

  ld.add_action(Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='map_to_odom_tf',
    output='screen',
    arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    condition=IfCondition(start_odin_driver)))

  # Real robot: attach the planner's synthetic sensor/vehicle tree to ODIN's
  # live odometry tree. ODIN is mounted forward-facing with zero roll/pitch/yaw
  # offset, so an identity transform is the safest first approximation.
  ld.add_action(Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='odin_base_to_planner_sensor_tf',
    output='screen',
    arguments=['0', '0', '0', '0', '0', '0',
               odin_base_frame, planner_sensor_frame],
    condition=IfCondition(start_odin_driver)))

  ld.add_action(Node(
    package='joy',
    executable='joy_node',
    name='ps3_joy',
    output='screen',
    parameters=[{'dev': '/dev/input/js0', 'deadzone': 0.12, 'autorepeat_rate': 0.0}]))

  ld.add_action(Node(
    package='odin_ros_driver',
    executable='host_sdk_sample',
    name='host_sdk_sample',
    output='screen',
    parameters=[{'config_file': odin_config_file}],
    condition=IfCondition(start_odin_driver)
  ))

  ld.add_action(IncludeLaunchDescription(
    PythonLaunchDescriptionSource([get_package_share_directory('odin_autonomy_bridge'),
                                   '/launch/odin_autonomy_bridge.launch.py']),
    launch_arguments={
      'input_cloud_topic': odin_cloud_topic,
      'cloud_transform_with_latest_odom': 'false',
    }.items(),
    condition=IfCondition(start_odin_driver)))

  ld.add_action(IncludeLaunchDescription(
    PythonLaunchDescriptionSource([get_package_share_directory('d1_compat_bridge'),
                                   '/launch/d1_compat_bridge.launch.py']),
    launch_arguments={
      'input_joint_states_topic': d1_joint_states_topic,
      'input_imu_topic': d1_imu_topic,
      'input_fsm_topic': d1_fsm_topic,
      'input_motors_status_topic': d1_motors_status_topic,
      'input_odom_topic': '',
      'output_fsm_topic': d1_key_topic,
      'output_state_estimation_topic': '',
      'standup_key': d1_standup_key,
      'standdown_key': d1_standdown_key,
      'publish_twist_from_motion_cmd': 'false',
    }.items()))

  delayed_actions = [
    IncludeLaunchDescription(
      FrontendLaunchDescriptionSource(os.path.join(
        get_package_share_directory('local_planner'), 'launch', 'local_planner.launch')),
      launch_arguments={
        'sensorOffsetX': sensorOffsetX,
        'sensorOffsetY': sensorOffsetY,
        'cameraOffsetZ': cameraOffsetZ,
        'goalX': vehicleX,
        'goalY': vehicleY,
        'autonomyMode': autonomy_mode,
        'autonomyOnWaypoint': 'true',
        'ignoreJoyAutonomySwitch': 'true',
        'ignoreJoyManualSwitch': 'true',
        'autonomySpeed': autonomy_speed,
        'cmdVelTopic': d1_cmd_topic,
      }.items()),
    IncludeLaunchDescription(
      FrontendLaunchDescriptionSource(os.path.join(
        get_package_share_directory('terrain_analysis'), 'launch', 'terrain_analysis.launch'))),
    IncludeLaunchDescription(
      FrontendLaunchDescriptionSource(os.path.join(
        get_package_share_directory('terrain_analysis_ext'), 'launch', 'terrain_analysis_ext.launch')),
      launch_arguments={'checkTerrainConn': checkTerrainConn}.items()),
    IncludeLaunchDescription(
      FrontendLaunchDescriptionSource(os.path.join(
        get_package_share_directory('sensor_scan_generation'), 'launch', 'sensor_scan_generation.launch'))),
    IncludeLaunchDescription(
      FrontendLaunchDescriptionSource(os.path.join(
        get_package_share_directory('visualization_tools'), 'launch', 'visualization_tools.launch')),
      launch_arguments={'world_name': world_name}.items()),
    IncludeLaunchDescription(
      FrontendLaunchDescriptionSource(os.path.join(
        get_package_share_directory('receive_theta'), 'launch', 'receive_theta.launch')),
      launch_arguments={'showImage': showImage}.items(),
      condition=IfCondition(start_receive_theta)),
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([get_package_share_directory('far_planner'),
                                     '/launch/far_planner.launch']),
      launch_arguments={'config': route_planner_config}.items()),
  ]

  ld.add_action(TimerAction(
    period=startup_delay_sec,
    actions=delayed_actions))

  return ld
