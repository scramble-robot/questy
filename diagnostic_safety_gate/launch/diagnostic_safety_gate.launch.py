from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('diagnostic_safety_gate')
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    # Standalone node version
    safety_gate_node = Node(
        package='diagnostic_safety_gate',
        executable='diagnostic_safety_gate_node',
        name='diagnostic_safety_gate',
        output='screen',
        parameters=[params_file],
        remappings=[
            # Input: subscribe to cmd_vel_in
            # Output: publish to cmd_vel
            # Diagnostics: subscribe to /diagnostics (global topic)
        ]
    )

    return LaunchDescription([
        safety_gate_node
    ])
