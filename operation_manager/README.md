# operation_manager

ROS2 package that subscribes to GPIO topics (published by `gpio_reader`) and decides whether GPIO IO is "controllable".

Publishers:
- `/gpio/controllable` (std_msgs/Bool): true if controllable
- `/gpio/controllable_diagnostic` (std_msgs/String): diagnostic message (timeout/false value)

Parameters (config/operation_manager.yaml):
- `monitored_pins`: list of GPIO pin numbers to subscribe to (`gpio_<PIN>` topics)
- `timeout_seconds`: maximum allowed age of GPIO update before marking not controllable

Build:
- colcon build --packages-select operation_manager

Run (with config):
- ros2 launch operation_manager operation_manager.launch.py
