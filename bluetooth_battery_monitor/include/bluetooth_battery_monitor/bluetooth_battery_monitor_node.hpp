#ifndef BLUETOOTH_BATTERY_MONITOR__BLUETOOTH_BATTERY_MONITOR_NODE_HPP_
#define BLUETOOTH_BATTERY_MONITOR__BLUETOOTH_BATTERY_MONITOR_NODE_HPP_

#include <array>
#include <chrono>
#include <memory>
#include <string>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_updater/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

namespace bluetooth_battery_monitor {

class BluetoothBatteryMonitor : public rclcpp::Node {
public:
  explicit BluetoothBatteryMonitor(const rclcpp::NodeOptions& options);

private:
  // Timer callback
  void timer_callback();

  // Execute upower command and parse output
  bool get_battery_info(double& percentage, std::string& state, bool& is_present);

  // Diagnostic updater callback
  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);

  // Helper function to execute shell command
  std::string execute_command(const std::string& cmd);

  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Diagnostic updater
  std::unique_ptr<diagnostic_updater::Updater> diagnostic_updater_;

  // Parameters
  std::string device_path_;
  double update_rate_;
  double low_battery_threshold_;
  double critical_battery_threshold_;

  // Battery state cache
  double last_percentage_;
  std::string last_state_;
  bool device_connected_;
};

}  // namespace bluetooth_battery_monitor

#endif  // BLUETOOTH_BATTERY_MONITOR__BLUETOOTH_BATTERY_MONITOR_NODE_HPP_
