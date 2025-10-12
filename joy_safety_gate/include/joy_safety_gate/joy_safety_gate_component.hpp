#ifndef JOY_SAFETY_GATE__JOY_SAFETY_GATE_COMPONENT_HPP_
#define JOY_SAFETY_GATE__JOY_SAFETY_GATE_COMPONENT_HPP_

#include <chrono>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <string>
#include <vector>

namespace joy_safety_gate {

class JoySafetyGate : public rclcpp::Node {
public:
  explicit JoySafetyGate(const rclcpp::NodeOptions& options);
  ~JoySafetyGate() = default;

private:
  void joyInputCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void diagnosticsCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);
  void checkDiagnosticsTimeout();
  bool isSafeToOperate() const;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_input_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;

  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_output_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // State variables
  bool is_safe_;
  std::chrono::steady_clock::time_point last_diagnostics_time_;
  std::chrono::steady_clock::time_point last_joy_time_;

  // Parameters
  double diagnostics_timeout_;              // seconds
  std::vector<std::string> monitor_names_;  // diagnostic names to monitor (empty = all)
  std::vector<std::string> block_levels_;   // diagnostic levels that trigger blocking
  bool require_diagnostics_;                // require diagnostics before allowing Joy messages
  bool pass_through_when_unsafe_;           // if true, always pass Joy; if false, block when unsafe
};

}  // namespace joy_safety_gate

#endif  // JOY_SAFETY_GATE__JOY_SAFETY_GATE_COMPONENT_HPP_
