import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot').perform(context)
    can_interface = LaunchConfiguration('can_interface').perform(context)
    controller_overrides = LaunchConfiguration('controller_overrides').perform(context)
    controller_name = LaunchConfiguration('controller_name').perform(context)
    start_vendor_teleop = LaunchConfiguration('start_vendor_teleop').perform(context)
    teleop_use_sdk = LaunchConfiguration('teleop_use_sdk').perform(context)
    teleop_debug = LaunchConfiguration('teleop_debug').perform(context)
    nn = os.environ.get('ROBOT_NS', '')

    robot_xacro_path = os.path.join(
        get_package_share_directory(robot_name + '_description'),
        'xacro',
        'robot.xacro',
    )
    robot_description = xacro.process_file(
        robot_xacro_path,
        mappings={'hw_env': 'hw', 'can_interface': can_interface},
    ).toxml()

    base_controller_config = os.path.join(
        get_package_share_directory('rl_controller'),
        'config',
        robot_name,
        'controllers.yaml',
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='both',
        parameters=[
            {'robot_description': robot_description},
            base_controller_config,
            controller_overrides,
            {'enable_command': True},
        ],
        namespace=nn,
    )

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'frame_prefix': nn + '/'},
        ],
        namespace=nn,
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            nn + '/controller_manager',
            '--controller-manager-timeout',
            '120',
            '--service-call-timeout',
            '60',
            '--switch-timeout',
            '60',
        ],
    )

    imu_sensor_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'imu_sensor_broadcaster',
            '--controller-manager',
            nn + '/controller_manager',
            '--controller-manager-timeout',
            '120',
            '--service-call-timeout',
            '60',
            '--switch-timeout',
            '60',
        ],
    )

    system_status_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'system_status_broadcaster',
            '--controller-manager',
            nn + '/controller_manager',
            '--controller-manager-timeout',
            '120',
            '--service-call-timeout',
            '60',
            '--switch-timeout',
            '60',
        ],
    )

    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            controller_name,
            '--controller-manager',
            nn + '/controller_manager',
            '--controller-manager-timeout',
            '120',
            '--service-call-timeout',
            '60',
            '--switch-timeout',
            '60',
        ],
    )

    teleop_config = os.path.join(
        get_package_share_directory('teleop_command'),
        'config',
        'param.yaml',
    )

    teleop_node = Node(
        package='teleop_command',
        executable='teleop_command_node',
        namespace=nn,
        output='screen',
        parameters=[
            teleop_config,
            {
                'can_interface': can_interface,
                'use_sdk': teleop_use_sdk.lower() == 'true',
                'debug': teleop_debug.lower() == 'true',
            },
        ],
    )

    actions = [
        control_node,
        robot_state_pub_node,
        TimerAction(period=2.0, actions=[joint_state_broadcaster_spawner]),
        TimerAction(period=4.0, actions=[imu_sensor_broadcaster_spawner]),
        TimerAction(period=6.0, actions=[system_status_broadcaster_spawner]),
        TimerAction(period=8.0, actions=[controller_spawner]),
    ]

    if start_vendor_teleop.lower() == 'true':
        actions.append(TimerAction(period=10.0, actions=[teleop_node]))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='d1'),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('controller_name', default_value='d1_rl_controller'),
        DeclareLaunchArgument('start_vendor_teleop', default_value='true'),
        DeclareLaunchArgument('teleop_use_sdk', default_value='false'),
        DeclareLaunchArgument('teleop_debug', default_value='false'),
        DeclareLaunchArgument(
            'controller_overrides',
            default_value=os.path.join(
                get_package_share_directory('d1_compat_bridge'),
                'config',
                'd1_lqr_overrides.yaml',
            ),
        ),
        OpaqueFunction(function=launch_setup),
    ])
