from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('input_odom_topic', default_value='/odin1/odometry'),
        DeclareLaunchArgument('input_cloud_topic', default_value='/odin1/cloud_slam'),
        DeclareLaunchArgument('input_imu_topic', default_value='/odin1/imu'),
        DeclareLaunchArgument('output_state_estimation_topic', default_value='/state_estimation'),
        DeclareLaunchArgument('output_registered_scan_topic', default_value='/registered_scan'),
        DeclareLaunchArgument('output_imu_topic', default_value='/imu/data'),
        DeclareLaunchArgument('world_frame', default_value='map'),
        DeclareLaunchArgument('rewrite_frame_id', default_value='true'),
        DeclareLaunchArgument('cloud_transform_with_latest_odom', default_value='false'),
        Node(
            package='odin_autonomy_bridge',
            executable='odin_autonomy_bridge_node',
            name='odin_autonomy_bridge_odom',
            output='screen',
            parameters=[{
                'input_odom_topic': LaunchConfiguration('input_odom_topic'),
                'input_cloud_topic': LaunchConfiguration('input_cloud_topic'),
                'input_imu_topic': LaunchConfiguration('input_imu_topic'),
                'output_state_estimation_topic': LaunchConfiguration('output_state_estimation_topic'),
                'output_registered_scan_topic': LaunchConfiguration('output_registered_scan_topic'),
                'output_imu_topic': LaunchConfiguration('output_imu_topic'),
                'world_frame': LaunchConfiguration('world_frame'),
                'rewrite_frame_id': LaunchConfiguration('rewrite_frame_id'),
                'cloud_transform_with_latest_odom': False,
                'enable_odom_bridge': True,
                'enable_cloud_bridge': False,
                'enable_imu_bridge': False,
            }]
        ),
        Node(
            package='odin_autonomy_bridge',
            executable='odin_autonomy_bridge_node',
            name='odin_autonomy_bridge_cloud',
            output='screen',
            parameters=[{
                'input_odom_topic': LaunchConfiguration('input_odom_topic'),
                'input_cloud_topic': LaunchConfiguration('input_cloud_topic'),
                'input_imu_topic': LaunchConfiguration('input_imu_topic'),
                'output_state_estimation_topic': LaunchConfiguration('output_state_estimation_topic'),
                'output_registered_scan_topic': LaunchConfiguration('output_registered_scan_topic'),
                'output_imu_topic': LaunchConfiguration('output_imu_topic'),
                'world_frame': LaunchConfiguration('world_frame'),
                'rewrite_frame_id': LaunchConfiguration('rewrite_frame_id'),
                'cloud_transform_with_latest_odom': LaunchConfiguration('cloud_transform_with_latest_odom'),
                'enable_odom_bridge': False,
                'enable_cloud_bridge': True,
                'enable_imu_bridge': False,
            }]
        ),
        Node(
            package='odin_autonomy_bridge',
            executable='odin_autonomy_bridge_node',
            name='odin_autonomy_bridge_imu',
            output='screen',
            parameters=[{
                'input_odom_topic': LaunchConfiguration('input_odom_topic'),
                'input_cloud_topic': LaunchConfiguration('input_cloud_topic'),
                'input_imu_topic': LaunchConfiguration('input_imu_topic'),
                'output_state_estimation_topic': LaunchConfiguration('output_state_estimation_topic'),
                'output_registered_scan_topic': LaunchConfiguration('output_registered_scan_topic'),
                'output_imu_topic': LaunchConfiguration('output_imu_topic'),
                'world_frame': LaunchConfiguration('world_frame'),
                'rewrite_frame_id': LaunchConfiguration('rewrite_frame_id'),
                'cloud_transform_with_latest_odom': False,
                'enable_odom_bridge': False,
                'enable_cloud_bridge': False,
                'enable_imu_bridge': True,
            }]
        )
    ])
