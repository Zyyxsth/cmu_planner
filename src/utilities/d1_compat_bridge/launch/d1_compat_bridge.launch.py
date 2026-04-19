from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('input_motion_cmd_topic', default_value='diablo/MotionCmd'),
        DeclareLaunchArgument('input_joint_states_topic', default_value='joint_states'),
        DeclareLaunchArgument('input_imu_topic', default_value='imu_sensor_broadcaster/imu'),
        DeclareLaunchArgument('input_fsm_topic', default_value='rl_controller/fsm'),
        DeclareLaunchArgument('input_motors_status_topic', default_value='system_status_broadcaster/motors_status'),
        DeclareLaunchArgument('input_odom_topic', default_value=''),
        DeclareLaunchArgument('output_twist_topic', default_value='command/cmd_twist'),
        DeclareLaunchArgument('output_fsm_topic', default_value='command/cmd_key'),
        DeclareLaunchArgument('output_state_estimation_topic', default_value=''),
        DeclareLaunchArgument('output_robot_status_topic', default_value='diablo/sensor/Body_state'),
        DeclareLaunchArgument('output_leg_motors_topic', default_value='diablo/sensor/Motors'),
        DeclareLaunchArgument('output_imu_euler_topic', default_value='/diablo/sensor/ImuEuler'),
        DeclareLaunchArgument('vehicle_frame', default_value='vehicle'),
        DeclareLaunchArgument('standup_key', default_value='transform_up'),
        DeclareLaunchArgument('standdown_key', default_value='transform_down'),
        DeclareLaunchArgument('nominal_leg_length', default_value='0.40'),
        DeclareLaunchArgument('publish_twist_from_motion_cmd', default_value='true'),
        Node(
            package='d1_compat_bridge',
            executable='d1_compat_bridge_node',
            output='screen',
            parameters=[{
                'input_motion_cmd_topic': LaunchConfiguration('input_motion_cmd_topic'),
                'input_joint_states_topic': LaunchConfiguration('input_joint_states_topic'),
                'input_odom_topic': LaunchConfiguration('input_odom_topic'),
                'input_imu_topic': LaunchConfiguration('input_imu_topic'),
                'input_fsm_topic': LaunchConfiguration('input_fsm_topic'),
                'input_motors_status_topic': LaunchConfiguration('input_motors_status_topic'),
                'output_twist_topic': LaunchConfiguration('output_twist_topic'),
                'output_fsm_topic': LaunchConfiguration('output_fsm_topic'),
                'output_state_estimation_topic': LaunchConfiguration('output_state_estimation_topic'),
                'output_robot_status_topic': LaunchConfiguration('output_robot_status_topic'),
                'output_leg_motors_topic': LaunchConfiguration('output_leg_motors_topic'),
                'output_imu_euler_topic': LaunchConfiguration('output_imu_euler_topic'),
                'vehicle_frame': LaunchConfiguration('vehicle_frame'),
                'standup_key': LaunchConfiguration('standup_key'),
                'standdown_key': LaunchConfiguration('standdown_key'),
                'nominal_leg_length': LaunchConfiguration('nominal_leg_length'),
                'publish_twist_from_motion_cmd': LaunchConfiguration('publish_twist_from_motion_cmd'),
            }]
        )
    ])
