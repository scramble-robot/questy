# Joy Gate

Joy Gate is a ROS2 package that provides safety gating functionality for joystick messages based on GPIO controllable status.

## Overview

The Joy Gate node monitors the `/gpio/controllable` topic and gates joystick messages accordingly:
- When `/gpio/controllable` is `true`: Joy messages from `/joy` are passed through to `/joy_gated`
- When `/gpio/controllable` is `false`: Zero messages (all axes and buttons set to 0) are published to `/joy_gated`

## Features

- **Safety First**: Automatically stops joy commands when GPIO controllable status is false
- **Message Structure Preservation**: Maintains the same message structure as input joy messages
- **Real-time Switching**: Immediate response to GPIO controllable status changes
- **Component Architecture**: Supports both standalone and composable node execution
- **Debug Logging**: Detailed logging for troubleshooting

## Topics

### Subscribed Topics
- `/gpio/controllable` (`std_msgs/msg/Bool`): GPIO controllable status
- `/joy` (`sensor_msgs/msg/Joy`): Input joystick messages

### Published Topics
- `/joy_gated` (`sensor_msgs/msg/Joy`): Gated joystick messages

## Usage

### Run Joy Gate Node Only
```bash
ros2 launch joy_gate joy_gate.launch.py
```

### Run Complete System (GPIO Reader + Joy Controller + Joy Gate)
```bash
ros2 launch joy_gate joy_gate_full.launch.py
```

### Launch Arguments
- `use_sim_time`: Use simulation time (default: false)
- `log_level`: Logging level (default: info)
- `joy_device`: Joystick device path (default: /dev/input/js0) [full launch only]
- `gpio_config`: GPIO reader config file [full launch only]
- `joy_config`: Joy controller config file [full launch only]

## Building

This package uses `ament_cmake_auto` for simplified CMake configuration:

```bash
cd /path/to/workspace
colcon build --packages-select joy_gate
source install/setup.bash
```

## Dependencies

- rclcpp
- rclcpp_components
- sensor_msgs
- std_msgs

## Testing

To test the functionality:

1. Launch the complete system:
   ```bash
   ros2 launch joy_gate joy_gate_full.launch.py
   ```

2. Monitor the gated output:
   ```bash
   ros2 topic echo /joy_gated
   ```

3. Change GPIO controllable status:
   ```bash
   # Allow control
   ros2 topic pub /gpio/controllable std_msgs/msg/Bool "{data: true}" --once
   
   # Block control
   ros2 topic pub /gpio/controllable std_msgs/msg/Bool "{data: false}" --once
   ```

4. Move the joystick and observe that messages are only passed through when controllable is true.