#include "diagnostic_safety_gate/diagnostic_safety_gate_component.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace diagnostic_safety_gate {

DiagnosticSafetyGate::DiagnosticSafetyGate(const rclcpp::NodeOptions& options)
    : Node("diagnostic_safety_gate", options), is_safe_(false) {
  // Declare parameters
  this->declare_parameter("diagnostics_timeout", 5.0);
  this->declare_parameter("cmd_vel_timeout", 0.5);
  this->declare_parameter("monitor_names", std::vector<std::string>{});
  this->declare_parameter("block_levels", std::vector<std::string>{"ERROR"});
  this->declare_parameter("require_diagnostics", true);

  // Get parameters
  diagnostics_timeout_ = this->get_parameter("diagnostics_timeout").as_double();
  cmd_vel_timeout_ = this->get_parameter("cmd_vel_timeout").as_double();
  monitor_names_ = this->get_parameter("monitor_names").as_string_array();
  block_levels_ = this->get_parameter("block_levels").as_string_array();
  require_diagnostics_ = this->get_parameter("require_diagnostics").as_bool();

  // Create subscribers
  cmd_vel_input_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_in", 10,
      std::bind(&DiagnosticSafetyGate::cmdVelInputCallback, this, std::placeholders::_1));

  diagnostics_sub_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10,
      std::bind(&DiagnosticSafetyGate::diagnosticsCallback, this, std::placeholders::_1));

  // Create publisher
  cmd_vel_output_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  // Create timer for periodic checks (10 Hz)
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                   std::bind(&DiagnosticSafetyGate::checkTimeoutAndPublish, this));

  // Initialize timestamps
  last_diagnostics_time_ = std::chrono::steady_clock::now();
  last_cmd_vel_time_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(this->get_logger(), "Diagnostic Safety Gate initialized");
  RCLCPP_INFO(this->get_logger(), "  diagnostics_timeout: %.2f s", diagnostics_timeout_);
  RCLCPP_INFO(this->get_logger(), "  cmd_vel_timeout: %.2f s", cmd_vel_timeout_);
  RCLCPP_INFO(this->get_logger(), "  require_diagnostics: %s",
              require_diagnostics_ ? "true" : "false");

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

void DiagnosticSafetyGate::cmdVelInputCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  last_cmd_vel_ = msg;
  last_cmd_vel_time_ = std::chrono::steady_clock::now();

  // Immediately check if we can pass through the command
  if (isSafeToOperate()) {
    cmd_vel_output_pub_->publish(*msg);
    RCLCPP_DEBUG(this->get_logger(), "Command passed through (safe)");
  } else {
    // Publish zero velocity when not safe
    auto zero_cmd = geometry_msgs::msg::Twist();
    cmd_vel_output_pub_->publish(zero_cmd);
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(),
                         1000,  // Log once per second
                         "Command blocked due to unsafe diagnostics state");
  }
}

void DiagnosticSafetyGate::diagnosticsCallback(
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
                               2000,  // Log once per 2 seconds
                               "Diagnostic '%s' has level %s - blocking motor commands",
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

void DiagnosticSafetyGate::checkTimeoutAndPublish() {
  auto now = std::chrono::steady_clock::now();

  // Check diagnostics timeout
  if (require_diagnostics_) {
    auto diag_elapsed = std::chrono::duration<double>(now - last_diagnostics_time_).count();
    if (diag_elapsed > diagnostics_timeout_) {
      if (is_safe_) {
        RCLCPP_WARN(this->get_logger(), "Diagnostics timeout (%.2f s) - blocking motor commands",
                    diag_elapsed);
        is_safe_ = false;
      }
    }
  }

  // Check cmd_vel timeout and publish zero if timed out
  auto cmd_elapsed = std::chrono::duration<double>(now - last_cmd_vel_time_).count();
  if (cmd_elapsed > cmd_vel_timeout_ && last_cmd_vel_) {
    // Timeout - publish zero velocity
    auto zero_cmd = geometry_msgs::msg::Twist();
    cmd_vel_output_pub_->publish(zero_cmd);
    last_cmd_vel_.reset();  // Clear the saved command
  }
}

bool DiagnosticSafetyGate::isSafeToOperate() const { return is_safe_; }

}  // namespace diagnostic_safety_gate

RCLCPP_COMPONENTS_REGISTER_NODE(diagnostic_safety_gate::DiagnosticSafetyGate)
