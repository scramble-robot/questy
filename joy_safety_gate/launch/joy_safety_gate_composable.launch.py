from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Launch Joy Safety Gate as a composable node in a container."""
    
    pkg_dir = get_package_share_directory('joy_safety_gate')
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    container = ComposableNodeContainer(
        name='joy_safety_gate_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='joy_safety_gate',
                plugin='joy_safety_gate::JoySafetyGate',
                name='joy_safety_gate',
                parameters=[params_file],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])
