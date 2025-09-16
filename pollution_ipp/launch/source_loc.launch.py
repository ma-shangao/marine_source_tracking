# Copyright 2025 author
import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    start_client_arg = DeclareLaunchArgument(
        'start_client',
        default_value='false',
        description='whether to start the source location client'
    )

    record_odom_arg = DeclareLaunchArgument(
        'record_odom',
        default_value='false',
        description='whether to record odometry data'
    )

    scenario = LaunchConfiguration('scenario')
    scenario_arg = DeclareLaunchArgument(
        'scenario',
        default_value='S2',
        description='Scenario configuration'
    )

    st = LaunchConfiguration('st')
    st_arg = DeclareLaunchArgument(
        'st',
        default_value='st1',
        description='Scenario trial starting point'
    )

    config_path = LaunchConfiguration('config_path')
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value=os.path.join(
            os.path.expanduser('~'),
            'experiments'
        ),
        description='Path to the scenario configuration files'
    )

    rviz_config = os.path.join(
        get_package_share_directory('pollution_ipp'),
        'config',
        'prob_map.rviz'
    )

    config_yaml_file = ParameterFile(
        param_file=PathJoinSubstitution([
            config_path,
            scenario,
            st,
            'ros_param.yaml'
        ]),
        allow_substs=True
    )

    source_loc_server = Node(
        package='pollution_ipp',
        executable='source_loc_node',
        name='source_loc_node',
        parameters=[config_yaml_file]
    )

    manual_control = Node(
        package='pollution_ipp',
        executable='rviz_goal_pose_node',
        name='rviz_goal_pose_node'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': True
        }],
    )

    source_loc_client = Node(
        package='pollution_ipp',
        executable='source_loc_client',
        name='source_loc_client',
        parameters=[{
            'use_sim_time': True,
            'ci_threshold': 10.0,
            'output_file': 'exp/source_loc_estimates.csv',
            'append_timestamp': True,
            'recording_rate_hz': 1.0,  # Record at 1 Hz
        }],
        condition=IfCondition(LaunchConfiguration('start_client'))
    )

    # Include odometry recorder with conditional launch
    odom_recorder = Node(
        package='pollution_ipp',
        executable='odometry_recorder',
        name='odometry_recorder',
        parameters=[{
            'use_sim_time': True,
            'output_file': 'exp/odometry_data.csv',
            'recording_rate_hz': 2.0,
        }],
        condition=IfCondition(LaunchConfiguration('record_odom'))
    )

    return LaunchDescription([
            start_client_arg,
            record_odom_arg,
            scenario_arg,
            st_arg,
            config_path_arg,
            source_loc_server,
            rviz,
            manual_control,
            source_loc_client,
            odom_recorder
    ])
