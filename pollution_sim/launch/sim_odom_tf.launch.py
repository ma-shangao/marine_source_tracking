# Copyright 2025 author
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():

    # scenarios = {
    #     'a': 'S2',
    #     'b': 'S3'
    # }

    scenario = LaunchConfiguration('scenario')
    scenario_arg = DeclareLaunchArgument(
        'scenario',
        default_value='S2',
        description='Scenario configuration'
    )

    gz_bridge_config_file = os.path.join(
        get_package_share_directory('pollution_sim'),
        'config',
        'ros_gz_bridge.yaml'
    )

    gz_bridgde = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        parameters=[{
            'config_file': gz_bridge_config_file
        }],
    )

    p3d_odom_to_tf = Node(
        package='odom_to_tf_ros2',
        executable='odom_to_tf',
        name='p3d_odom_to_tf',
        parameters=[{
            'odom_topic': '/blueboat/odometry',
            # 'frame_id': 'base_link',
            'child_frame_id': 'ground_truth',
            'inverse_tf': True,
            'use_sim_time': True
        }],
    )

    ap_pose_to_tf = Node(
        package='odom_to_tf_ros2',
        executable='odom_to_tf',
        name='ap_pose_to_tf',
        parameters=[{
            'odom_topic': '/ap/pose/filtered',
            'frame_id': 'odom',
            'inverse_tf': False,
            'use_sim_time': True
        }],
    )

    pollution_sim_csv = Node(
        package='pollution_sim',
        executable='pollution_sim_node_csv',
        name='pollution_sim_node_csv',
        parameters=[{
            'use_sim_time': True,
            'time_step_num': 600,
            'csv_dir_path': PathJoinSubstitution([
                os.path.expanduser('~'),
                'dispersion_results',
                scenario,
                'dispersion'
            ])
        }],
    )

    return LaunchDescription([
        scenario_arg,
        gz_bridgde,
        RegisterEventHandler(
            OnProcessStart(
                target_action=gz_bridgde,
                on_start=[
                    LogInfo(
                        msg='gz_bridge is starting, starting p3d_odom_to_tf and ap_pose_to_tf'
                    ),
                    TimerAction(
                        period=5.0,
                        actions=[p3d_odom_to_tf, ap_pose_to_tf, pollution_sim_csv]
                    )
                ]
            )
        )
    ])
