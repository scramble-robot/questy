# Bluetooth Battery Monitor

ROS2 component for monitoring Bluetooth device battery status using UPower with diagnostic_updater integration.

## Features

- 🔋 Monitor Bluetooth device battery level via `upower`
- 📊 Publish battery state as `sensor_msgs/BatteryState`
- 🩺 Integrate with `diagnostic_updater` for health monitoring
- ⚡ Low CPU overhead with configurable polling intervals
- 🔌 ROS2 component-based architecture for flexible deployment

## Dependencies

- ROS2 (Humble/Jazzy)
- `upower` system package
- Bluetooth device that reports battery status

## Installation

```bash
# Install upower if not already installed
sudo apt install upower

# Build the package
cd ~/your_ws
colcon build --packages-select bluetooth_battery_monitor
source install/setup.bash
```

## Usage

### Finding Your Device Path

First, find your Bluetooth device path:

```bash
upower -e
```

Look for entries like:

```
/org/freedesktop/UPower/devices/headset_dev_XX_XX_XX_XX_XX_XX
/org/freedesktop/UPower/devices/mouse_dev_YY_YY_YY_YY_YY_YY
```

Get detailed info about a device:

```bash
upower -i /org/freedesktop/UPower/devices/headset_dev_XX_XX_XX_XX_XX_XX
```

### Configuration

Edit `config/params.yaml` to set your device:

```yaml
/**:
  ros__parameters:
    device_path: "/org/freedesktop/UPower/devices/headset_dev_XX_XX_XX_XX_XX_XX"
    update_rate: 30.0  # Check battery every 30 seconds
    low_battery_threshold: 20.0
    critical_battery_threshold: 10.0
```

### Launching

```bash
# Launch with default config
ros2 launch bluetooth_battery_monitor bluetooth_battery_monitor.launch.py

# Launch with custom config
ros2 launch bluetooth_battery_monitor bluetooth_battery_monitor.launch.py \
  params_file:=/path/to/your/params.yaml
```

### Loading as a Component

```python
from launch_ros.descriptions import ComposableNode

ComposableNode(
    package='bluetooth_battery_monitor',
    plugin='bluetooth_battery_monitor::BluetoothBatteryMonitor',
    name='bluetooth_battery_monitor',
    parameters=[params_file]
)
```

## Topics

### Published

- `~/battery_state` (`sensor_msgs/BatteryState`) - Battery status information
  - `percentage`: Battery level (0.0 to 1.0)
  - `power_supply_status`: Charging/Discharging/Full/Unknown
  - `present`: Device connection status

## Diagnostics

The node publishes diagnostic information via `diagnostic_updater`:

- **OK**: Battery level above low threshold
- **WARN**: Battery level below low threshold (default: 20%)
- **ERROR**: Battery level below critical threshold (default: 10%) or device disconnected

View diagnostics:

```bash
ros2 topic echo /diagnostics
```

Or use rqt:

```bash
rqt_runtime_monitor
```

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `device_path` | string | (see config) | UPower device path |
| `update_rate` | double | 30.0 | Polling interval in seconds |
| `low_battery_threshold` | double | 20.0 | Warning threshold (%) |
| `critical_battery_threshold` | double | 10.0 | Critical threshold (%) |

## Performance

- CPU usage: < 0.1% with 30s polling interval
- Memory: ~5-10 MB
- Network: Minimal (only publishes on state change)

## Troubleshooting

### Device not found

```bash
# Check if device is detected by UPower
upower -e

# Verify device is connected via Bluetooth
bluetoothctl devices
```

### No battery information

Some Bluetooth devices don't report battery status to UPower. Verify with:

```bash
upower -i /org/freedesktop/UPower/devices/your_device_path
```

Look for `percentage:` and `state:` fields.

## License

Apache-2.0
