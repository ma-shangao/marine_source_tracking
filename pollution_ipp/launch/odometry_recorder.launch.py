from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'output_file',
            default_value='exp/odometry_data.csv',
            description='Output CSV filename'
        ),

        DeclareLaunchArgument(
            'append_timestamp',
            default_value='false',
            description='Whether to append timestamp to filename'
        ),

        Node(
            package='pollution_ipp',
            executable='odometry_recorder',
            name='odometry_recorder',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'output_file': LaunchConfiguration('output_file'),
                'append_timestamp': LaunchConfiguration('append_timestamp'),
                'recording_rate_hz': 2.0,
            }]
        ),
    ])
