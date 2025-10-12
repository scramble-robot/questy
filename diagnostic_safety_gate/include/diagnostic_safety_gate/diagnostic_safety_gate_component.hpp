#ifndef DIAGNOSTIC_SAFETY_GATE__DIAGNOSTIC_SAFETY_GATE_COMPONENT_HPP_
#define DIAGNOSTIC_SAFETY_GATE__DIAGNOSTIC_SAFETY_GATE_COMPONENT_HPP_

#include <chrono>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace diagnostic_safety_gate {

class DiagnosticSafetyGate : public rclcpp::Node {
public:
  explicit DiagnosticSafetyGate(const rclcpp::NodeOptions& options);
  ~DiagnosticSafetyGate() = default;

private:
  void cmdVelInputCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void diagnosticsCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);
  void checkTimeoutAndPublish();
  bool isSafeToOperate() const;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_input_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;

  // Publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_output_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // State variables
  geometry_msgs::msg::Twist::SharedPtr last_cmd_vel_;
  bool is_safe_;
  std::chrono::steady_clock::time_point last_diagnostics_time_;
  std::chrono::steady_clock::time_point last_cmd_vel_time_;

  // Parameters
  double diagnostics_timeout_;              // seconds
  double cmd_vel_timeout_;                  // seconds
  std::vector<std::string> monitor_names_;  // diagnostic names to monitor
  std::vector<std::string> block_levels_;   // diagnostic levels that trigger blocking (ERROR, WARN)
  bool require_diagnostics_;                // require diagnostics before allowing commands
};

}  // namespace diagnostic_safety_gate

#endif  // DIAGNOSTIC_SAFETY_GATE__DIAGNOSTIC_SAFETY_GATE_COMPONENT_HPP_
