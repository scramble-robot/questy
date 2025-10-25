#ifndef GPIO_SAFETY_GATE__GPIO_SAFETY_GATE_COMPONENT_HPP_
#define GPIO_SAFETY_GATE__GPIO_SAFETY_GATE_COMPONENT_HPP_

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace gpio_safety_gate {

class GpioSafetyGateComponent : public rclcpp::Node {
public:
  explicit GpioSafetyGateComponent(const rclcpp::NodeOptions& options);
  virtual ~GpioSafetyGateComponent();

private:
  void gpio_callback(const std_msgs::msg::Bool::SharedPtr msg, unsigned int pin);
  void check_safety_state();
  bool evaluate_safety_condition();

  // Parameters
  std::vector<int64_t> required_high_pins_;
  std::vector<int64_t> required_low_pins_;
  std::vector<int64_t> any_high_pins_;
  double timeout_duration_;
  std::string logic_mode_;  // "AND", "OR", "CUSTOM"
  bool default_safe_state_;

  // GPIO state tracking
  std::map<unsigned int, bool> gpio_states_;
  std::map<unsigned int, std::chrono::steady_clock::time_point> last_update_times_;

  // Current safety state
  bool is_safe_;
  bool all_pins_initialized_;

  // Subscribers and Publishers
  std::map<unsigned int, rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> gpio_subs_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safety_state_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr check_timer_;
};

}  // namespace gpio_safety_gate

#endif  // GPIO_SAFETY_GATE__GPIO_SAFETY_GATE_COMPONENT_HPP_
