#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('bluetooth_battery_monitor')
    
    # Launch arguments
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('bluetooth_battery_monitor'),
            'config',
            'params.yaml'
        ]),
        description='Path to the parameters file'
    )

    use_composition_arg = DeclareLaunchArgument(
        'use_composition',
        default_value='true',
        description='Whether to use component composition'
    )

    # Component container with the bluetooth battery monitor node
    container = ComposableNodeContainer(
        name='bluetooth_battery_monitor_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='bluetooth_battery_monitor',
                plugin='bluetooth_battery_monitor::BluetoothBatteryMonitor',
                name='bluetooth_battery_monitor',
                parameters=[LaunchConfiguration('params_file')],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        params_file_arg,
        use_composition_arg,
        container,
    ])
