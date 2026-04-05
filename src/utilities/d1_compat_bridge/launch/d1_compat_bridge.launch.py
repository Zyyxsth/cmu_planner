from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('input_motion_cmd_topic', default_value='diablo/MotionCmd'),
        DeclareLaunchArgument('input_odom_topic', default_value='/d1/odom'),
        DeclareLaunchArgument('input_imu_topic', default_value='/d1/imu'),
        DeclareLaunchArgument('output_twist_topic', default_value='/d1/cmd_vel'),
        DeclareLaunchArgument('output_twist_stamped_topic', default_value='/d1/cmd_vel_stamped'),
        DeclareLaunchArgument('output_state_estimation_topic', default_value='/state_estimation'),
        DeclareLaunchArgument('output_robot_status_topic', default_value='diablo/sensor/Body_state'),
        DeclareLaunchArgument('output_leg_motors_topic', default_value='diablo/sensor/Motors'),
        DeclareLaunchArgument('output_imu_euler_topic', default_value='/diablo/sensor/ImuEuler'),
        DeclareLaunchArgument('vehicle_frame', default_value='vehicle'),
        DeclareLaunchArgument('nominal_leg_length', default_value='0.40'),
        Node(
            package='d1_compat_bridge',
            executable='d1_compat_bridge_node',
            output='screen',
            parameters=[{
                'input_motion_cmd_topic': LaunchConfiguration('input_motion_cmd_topic'),
                'input_odom_topic': LaunchConfiguration('input_odom_topic'),
                'input_imu_topic': LaunchConfiguration('input_imu_topic'),
                'output_twist_topic': LaunchConfiguration('output_twist_topic'),
                'output_twist_stamped_topic': LaunchConfiguration('output_twist_stamped_topic'),
                'output_state_estimation_topic': LaunchConfiguration('output_state_estimation_topic'),
                'output_robot_status_topic': LaunchConfiguration('output_robot_status_topic'),
                'output_leg_motors_topic': LaunchConfiguration('output_leg_motors_topic'),
                'output_imu_euler_topic': LaunchConfiguration('output_imu_euler_topic'),
                'vehicle_frame': LaunchConfiguration('vehicle_frame'),
                'nominal_leg_length': LaunchConfiguration('nominal_leg_length'),
            }]
        )
    ])
