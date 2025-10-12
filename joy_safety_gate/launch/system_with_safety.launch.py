from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Complete system launch with Joy Safety Gate integration.
    
    System flow:
    joy_node -> joy_safety_gate -> joy_controller -> motor_control
                     ↑
                /diagnostics (from battery_monitor, etc.)
    """
    
    joy_safety_pkg = get_package_share_directory('joy_safety_gate')
    params_file = os.path.join(joy_safety_pkg, 'config', 'params.yaml')

    # Joy node (publishes to /joy)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }],
        output='screen'
    )

    # Joy Safety Gate (filters /joy based on /diagnostics)
    joy_safety_gate = Node(
        package='joy_safety_gate',
        executable='joy_safety_gate_node',
        name='joy_safety_gate',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('joy_in', '/joy'),           # Input from joy_node
            ('joy_out', '/joy_filtered'), # Output to joy_controller
        ]
    )

    # Joy Controller (converts /joy_filtered to /target_twist)
    joy_controller = Node(
        package='joy_controller',
        executable='joy_controller_node',
        name='joy_controller',
        output='screen',
        remappings=[
            ('/joy', '/joy_filtered'),    # Input from joy_safety_gate
            # Output: /target_twist (default)
        ]
    )

    # Motor Control App (receives /target_twist)
    # Uncomment and configure as needed for your motor control node
    # motor_control = Node(
    #     package='motor_control_app',
    #     executable='drive_node',
    #     name='motor_control',
    #     output='screen',
    #     remappings=[
    #         # ('/target_twist', '/target_twist'),  # Already default
    #     ]
    # )

    return LaunchDescription([
        joy_node,
        joy_safety_gate,
        joy_controller,
        # motor_control,  # Add when ready
    ])
