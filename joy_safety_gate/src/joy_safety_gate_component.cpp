#include "joy_safety_gate/joy_safety_gate_component.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace joy_safety_gate {

JoySafetyGate::JoySafetyGate(const rclcpp::NodeOptions& options)
    : Node("joy_safety_gate", options), is_safe_(false) {
  // Declare parameters
  this->declare_parameter("diagnostics_timeout", 5.0);
  this->declare_parameter("monitor_names", std::vector<std::string>{});
  this->declare_parameter("block_levels", std::vector<std::string>{"ERROR"});
  this->declare_parameter("require_diagnostics", true);
  this->declare_parameter("pass_through_when_unsafe", false);

  // Get parameters
  diagnostics_timeout_ = this->get_parameter("diagnostics_timeout").as_double();
  monitor_names_ = this->get_parameter("monitor_names").as_string_array();
  block_levels_ = this->get_parameter("block_levels").as_string_array();
  require_diagnostics_ = this->get_parameter("require_diagnostics").as_bool();
  pass_through_when_unsafe_ = this->get_parameter("pass_through_when_unsafe").as_bool();

  // Create subscribers
  joy_input_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy_in", 10, std::bind(&JoySafetyGate::joyInputCallback, this, std::placeholders::_1));

  diagnostics_sub_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10,
      std::bind(&JoySafetyGate::diagnosticsCallback, this, std::placeholders::_1));

  // Create publisher
  joy_output_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("joy_out", 10);

  // Create timer for periodic diagnostics timeout check (10 Hz)
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                   std::bind(&JoySafetyGate::checkDiagnosticsTimeout, this));

  // Initialize timestamps
  last_diagnostics_time_ = std::chrono::steady_clock::now();
  last_joy_time_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(this->get_logger(), "Joy Safety Gate initialized");
  RCLCPP_INFO(this->get_logger(), "  diagnostics_timeout: %.2f s", diagnostics_timeout_);
  RCLCPP_INFO(this->get_logger(), "  require_diagnostics: %s",
              require_diagnostics_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "  pass_through_when_unsafe: %s",
              pass_through_when_unsafe_ ? "true" : "false");

  if (!monitor_names_.empty()) {
    RCLCPP_INFO(this->get_logger(), "  Monitoring %zu diagnostic(s):", monitor_names_.size());
    for (const auto& name : monitor_names_) {
      RCLCPP_INFO(this->get_logger(), "    - %s", name.c_str());
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "  Monitoring all diagnostics");
  }

  RCLCPP_INFO(this->get_logger(), "  Blocking on levels:");
  for (const auto& level : block_levels_) {
    RCLCPP_INFO(this->get_logger(), "    - %s", level.c_str());
  }
}

void JoySafetyGate::joyInputCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
  last_joy_time_ = std::chrono::steady_clock::now();

  if (isSafeToOperate() || pass_through_when_unsafe_) {
    // Safe状態、またはpass_through設定の場合は通過させる
    joy_output_pub_->publish(*msg);
    RCLCPP_DEBUG(this->get_logger(), "Joy message passed through (safe=%s)",
                 is_safe_ ? "true" : "false");
  } else {
    // Unsafe状態でpass_throughが無効の場合はブロック
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(),
                         2000,  // Log once per 2 seconds
                         "Joy message blocked due to unsafe diagnostics state");
  }
}

void JoySafetyGate::diagnosticsCallback(
    const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
  last_diagnostics_time_ = std::chrono::steady_clock::now();

  // Check diagnostic status
  bool was_safe = is_safe_;
  is_safe_ = true;  // Assume safe until proven otherwise

  for (const auto& status : msg->status) {
    // If monitor_names is empty, check all diagnostics
    // Otherwise, only check monitored diagnostics
    bool should_check = monitor_names_.empty();
    if (!should_check) {
      for (const auto& name : monitor_names_) {
        if (status.name == name) {
          should_check = true;
          break;
        }
      }
    }

    if (should_check) {
      // Convert level to string for comparison
      std::string level_str;
      switch (status.level) {
        case diagnostic_msgs::msg::DiagnosticStatus::OK:
          level_str = "OK";
          break;
        case diagnostic_msgs::msg::DiagnosticStatus::WARN:
          level_str = "WARN";
          break;
        case diagnostic_msgs::msg::DiagnosticStatus::ERROR:
          level_str = "ERROR";
          break;
        case diagnostic_msgs::msg::DiagnosticStatus::STALE:
          level_str = "STALE";
          break;
        default:
          level_str = "UNKNOWN";
      }

      // Check if this level should block operation
      for (const auto& block_level : block_levels_) {
        if (level_str == block_level) {
          is_safe_ = false;
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(),
                               3000,  // Log once per 3 seconds
                               "Diagnostic '%s' has level %s - blocking Joy messages",
                               status.name.c_str(), level_str.c_str());
          break;
        }
      }
    }
  }

  // Log state transitions
  if (was_safe && !is_safe_) {
    RCLCPP_WARN(this->get_logger(), "System state changed: SAFE -> UNSAFE");
  } else if (!was_safe && is_safe_) {
    RCLCPP_INFO(this->get_logger(), "System state changed: UNSAFE -> SAFE");
  }
}

void JoySafetyGate::checkDiagnosticsTimeout() {
  if (require_diagnostics_) {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration<double>(now - last_diagnostics_time_).count();

    if (elapsed > diagnostics_timeout_) {
      if (is_safe_) {
        RCLCPP_WARN(this->get_logger(), "Diagnostics timeout (%.2f s) - marking as unsafe",
                    elapsed);
        is_safe_ = false;
      }
    }
  }
}

bool JoySafetyGate::isSafeToOperate() const { return is_safe_; }

}  // namespace joy_safety_gate

RCLCPP_COMPONENTS_REGISTER_NODE(joy_safety_gate::JoySafetyGate)
