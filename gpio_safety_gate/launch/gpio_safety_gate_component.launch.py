from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('gpio_safety_gate'),
            'config',
            'gpio_safety_gate.yaml'
        ]),
        description='Path to the GPIO safety gate configuration file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # Create container with GPIO Safety Gate component
    container = ComposableNodeContainer(
        name='gpio_safety_gate_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='gpio_safety_gate',
                plugin='gpio_safety_gate::GpioSafetyGateComponent',
                name='gpio_safety_gate_node',
                parameters=[
                    LaunchConfiguration('config_file'),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        container,
    ])
