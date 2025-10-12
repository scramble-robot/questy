from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Launch Joy Safety Gate as a standalone node."""
    
    pkg_dir = get_package_share_directory('joy_safety_gate')
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    joy_safety_gate_node = Node(
        package='joy_safety_gate',
        executable='joy_safety_gate_node',
        name='joy_safety_gate',
        output='screen',
        parameters=[params_file],
        remappings=[
            # Default remappings - can be overridden in parent launch file
            # Input: subscribe to joy_in (remap /joy to joy_in if needed)
            # Output: publish to joy_out (remap joy_out to /joy if needed)
            # Diagnostics: subscribe to /diagnostics (global topic)
        ]
    )

    return LaunchDescription([
        joy_safety_gate_node
    ])
