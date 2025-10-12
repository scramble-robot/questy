from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Launch diagnostic safety gate as a composable node in a container."""
    
    pkg_dir = get_package_share_directory('diagnostic_safety_gate')
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    container = ComposableNodeContainer(
        name='diagnostic_safety_gate_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='diagnostic_safety_gate',
                plugin='diagnostic_safety_gate::DiagnosticSafetyGate',
                name='diagnostic_safety_gate',
                parameters=[params_file],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])
