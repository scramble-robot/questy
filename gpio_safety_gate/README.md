# GPIO Safety Gate

ROS2 package for determining operational safety state based on GPIO inputs from Raspberry Pi.

## Overview

This package monitors GPIO pin states and determines whether the system is in a safe operational state. It's designed to work with the `gpio_reader` package to create a safety interlock system.

## Features

- Multiple logic modes: AND, OR, CUSTOM
- Configurable GPIO pin monitoring
- Timeout detection for stale GPIO data
- Real-time safety state publishing
- Implemented as ROS2 component for composability

## Dependencies

- ROS2 Humble or later
- gpio_reader package (for GPIO input)

## Configuration

### Logic Modes

1. **AND Mode**: All conditions must be met
   - All `required_high_pins` must be HIGH
   - All `required_low_pins` must be LOW

2. **OR Mode**: At least one condition must be met
   - At least one pin from `any_high_pins` must be HIGH

3. **CUSTOM Mode**: Combination of AND and OR
   - All required pins must match (like AND mode)
   - AND at least one from `any_high_pins` must be HIGH

### Configuration Parameters

Edit `config/gpio_safety_gate.yaml`:

- `logic_mode`: "AND", "OR", or "CUSTOM"
- `required_high_pins`: List of GPIO pins that must be HIGH
- `required_low_pins`: List of GPIO pins that must be LOW
- `any_high_pins`: List of pins where at least one must be HIGH (OR/CUSTOM mode)
- `timeout_duration`: Maximum time (seconds) without GPIO update before becoming unsafe
- `default_safe_state`: Safety state before all pins are initialized

## Building

```bash
cd ~/workspace/shr_ws/questy
colcon build --packages-select gpio_safety_gate
source install/setup.bash
```

## Usage

### Running with gpio_reader

First, start the GPIO reader:

```bash
ros2 launch gpio_reader gpio_reader.launch.py
```

Then start the safety gate:

```bash
ros2 launch gpio_safety_gate gpio_safety_gate.launch.py
```

### Running as component

```bash
ros2 launch gpio_safety_gate gpio_safety_gate_component.launch.py
```

### With custom configuration

```bash
ros2 launch gpio_safety_gate gpio_safety_gate.launch.py config_file:=/path/to/custom_config.yaml
```

## Topics

### Subscribed Topics

- `gpio_<PIN>` (std_msgs/Bool): Individual GPIO pin states from gpio_reader

### Published Topics

- `safety_state` (std_msgs/Bool): Current safety state (true = safe, false = unsafe)

## Example Use Cases

### Emergency Stop Button

Monitor a physical emergency stop button (active HIGH when not pressed):

```yaml
logic_mode: "AND"
required_high_pins: [17]  # Emergency stop button
required_low_pins: []
timeout_duration: 0.5
default_safe_state: false
```

### Multiple Safety Switches

Require all safety switches to be in safe position:

```yaml
logic_mode: "AND"
required_high_pins: [17, 27, 22]  # All safety switches must be HIGH
required_low_pins: []
timeout_duration: 1.0
default_safe_state: false
```

### Enable Switch with Safety Interlock

Require an enable switch AND no emergency stop:

```yaml
logic_mode: "CUSTOM"
required_high_pins: [17]   # Enable switch
required_low_pins: [27]    # Emergency stop (LOW = not pressed)
any_high_pins: []
timeout_duration: 0.5
default_safe_state: false
```

## Integration Example

Use the safety state to gate motor control commands:

```python
# In your motor control node
self.safety_sub = self.create_subscription(
    Bool, 'safety_state', self.safety_callback, 10)

def safety_callback(self, msg):
    self.is_safe = msg.data
    if not self.is_safe:
        self.emergency_stop()
```

## License

MIT
