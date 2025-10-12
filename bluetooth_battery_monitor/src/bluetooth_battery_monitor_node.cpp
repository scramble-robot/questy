#include "bluetooth_battery_monitor/bluetooth_battery_monitor_node.hpp"

#include <cstdio>
#include <memory>
#include <regex>
#include <sstream>

#include "rclcpp_components/register_node_macro.hpp"

namespace bluetooth_battery_monitor {

BluetoothBatteryMonitor::BluetoothBatteryMonitor(const rclcpp::NodeOptions& options)
    : Node("bluetooth_battery_monitor", options),
      last_percentage_(0.0),
      last_state_("Unknown"),
      device_connected_(false) {
  // Declare parameters
  this->declare_parameter("device_path",
                          "/org/freedesktop/UPower/devices/headset_dev_XX_XX_XX_XX_XX_XX");
  this->declare_parameter("update_rate", 30.0);                 // seconds
  this->declare_parameter("low_battery_threshold", 20.0);       // percentage
  this->declare_parameter("critical_battery_threshold", 10.0);  // percentage

  // Get parameters
  device_path_ = this->get_parameter("device_path").as_string();
  update_rate_ = this->get_parameter("update_rate").as_double();
  low_battery_threshold_ = this->get_parameter("low_battery_threshold").as_double();
  critical_battery_threshold_ = this->get_parameter("critical_battery_threshold").as_double();

  RCLCPP_INFO(this->get_logger(), "Monitoring Bluetooth device: %s", device_path_.c_str());
  RCLCPP_INFO(this->get_logger(), "Update rate: %.1f seconds", update_rate_);
  RCLCPP_INFO(this->get_logger(), "Low battery threshold: %.1f%%", low_battery_threshold_);
  RCLCPP_INFO(this->get_logger(), "Critical battery threshold: %.1f%%",
              critical_battery_threshold_);

  // Create publisher
  battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>("~/battery_state", 10);

  // Setup diagnostic updater
  diagnostic_updater_ = std::make_unique<diagnostic_updater::Updater>(this);
  diagnostic_updater_->setHardwareID("bluetooth_device");
  diagnostic_updater_->add("Battery Status", this, &BluetoothBatteryMonitor::produce_diagnostics);

  // Create timer
  auto timer_period = std::chrono::duration<double>(update_rate_);
  timer_ =
      this->create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(timer_period),
                              std::bind(&BluetoothBatteryMonitor::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "Bluetooth Battery Monitor initialized");
}

void BluetoothBatteryMonitor::timer_callback() {
  double percentage = 0.0;
  std::string state;
  bool is_present = false;

  // Get battery info from upower
  if (!get_battery_info(percentage, state, is_present)) {
    RCLCPP_DEBUG(this->get_logger(), "Failed to get battery info");
    device_connected_ = false;
    diagnostic_updater_->force_update();
    return;
  }

  device_connected_ = is_present;
  last_percentage_ = percentage;
  last_state_ = state;

  // Publish battery state
  auto battery_msg = sensor_msgs::msg::BatteryState();
  battery_msg.header.stamp = this->now();
  battery_msg.header.frame_id = "bluetooth_battery";

  battery_msg.percentage = static_cast<float>(percentage / 100.0);
  battery_msg.present = is_present;

  // Map state string to power_supply_status
  if (state == "charging" || state == "pending-charge") {
    battery_msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
  } else if (state == "discharging" || state == "pending-discharge") {
    battery_msg.power_supply_status =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  } else if (state == "fully-charged") {
    battery_msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL;
  } else {
    battery_msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  }

  battery_msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  battery_msg.power_supply_technology =
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;

  battery_pub_->publish(battery_msg);

  // Update diagnostics
  diagnostic_updater_->force_update();

  RCLCPP_DEBUG(this->get_logger(), "Battery: %.1f%% (%s)", percentage, state.c_str());
}

bool BluetoothBatteryMonitor::get_battery_info(double& percentage, std::string& state,
                                               bool& is_present) {
  // Execute upower command for specific device
  std::string cmd = "upower -i " + device_path_ + " 2>/dev/null";
  std::string output = execute_command(cmd);

  if (output.empty()) {
    RCLCPP_DEBUG(this->get_logger(), "Device not found or upower command failed");
    return false;
  }

  // Parse output
  std::istringstream stream(output);
  std::string line;
  bool found_percentage = false;
  bool found_state = false;
  is_present = false;

  std::regex percentage_regex(R"(percentage:\s+(\d+(?:\.\d+)?)%)");
  std::regex state_regex(R"(state:\s+(\S+))");
  std::regex present_regex(R"(present:\s+yes)");

  while (std::getline(stream, line)) {
    std::smatch match;

    // Check if device is present
    if (std::regex_search(line, present_regex)) {
      is_present = true;
    }

    // Extract percentage
    if (std::regex_search(line, match, percentage_regex)) {
      percentage = std::stod(match[1].str());
      found_percentage = true;
    }

    // Extract state
    if (std::regex_search(line, match, state_regex)) {
      state = match[1].str();
      found_state = true;
    }
  }

  return found_percentage && found_state && is_present;
}

void BluetoothBatteryMonitor::produce_diagnostics(
    diagnostic_updater::DiagnosticStatusWrapper& stat) {
  if (!device_connected_) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Device not connected");
    stat.add("Battery Percentage", "N/A");
    stat.add("State", "Disconnected");
    return;
  }

  // Determine diagnostic level based on battery percentage
  if (last_percentage_ <= critical_battery_threshold_) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Critical battery level");
  } else if (last_percentage_ <= low_battery_threshold_) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Low battery level");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Battery level OK");
  }

  // Add diagnostic data
  stat.add("Battery Percentage", std::to_string(last_percentage_) + "%");
  stat.add("State", last_state_);
  stat.add("Device Path", device_path_);
}

std::string BluetoothBatteryMonitor::execute_command(const std::string& cmd) {
  std::array<char, 128> buffer;
  std::string result;

  std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd.c_str(), "r"), pclose);

  if (!pipe) {
    RCLCPP_ERROR(this->get_logger(), "Failed to execute command: %s", cmd.c_str());
    return "";
  }

  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    result += buffer.data();
  }

  return result;
}

}  // namespace bluetooth_battery_monitor

RCLCPP_COMPONENTS_REGISTER_NODE(bluetooth_battery_monitor::BluetoothBatteryMonitor)
