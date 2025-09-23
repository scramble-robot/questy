#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM0',
            description='Serial port for DDT motor communication'
        ),
        
        DeclareLaunchArgument(
            'motor_id',
            default_value='1',
            description='DDT motor ID'
        ),

        # Single DDT Motor Component Node
        Node(
            package='motor_control_app',
            executable='single_ddt_motor_node',
            name='single_ddt_motor_node',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'motor_id': LaunchConfiguration('motor_id'),
                'max_motor_rpm': 100,
                'velocity_scale_factor': 60.0,
                'baud_rate': 115200,
                'wheel_radius': 0.1,
                'status_publish_rate': 5.0,
                'watchdog_timeout': 1.0
            }],
            remappings=[
                ('cmd_vel', '/cmd_vel'),
                ('motor_status', '/motor_status')
            ]
        ),

        # Test publisher node (sends twist messages for testing)
        TimerAction(
            period=2.0,  # Wait 2 seconds before starting test publisher
            actions=[
                Node(
                    package='motor_control_app',
                    executable='simple_motor_app',
                    name='twist_test_publisher',
                    output='screen',
                    parameters=[{
                        'publish_rate': 2.0,  # Publish every 0.5 seconds
                        'test_sequence': True  # Enable test sequence mode
                    }],
                    remappings=[
                        ('cmd_vel', '/cmd_vel')
                    ]
                )
            ]
        )
    ])