#include "gpio_safety_gate/gpio_safety_gate_component.hpp"

namespace gpio_safety_gate {

GpioSafetyGateComponent::GpioSafetyGateComponent(const rclcpp::NodeOptions& options)
    : Node("gpio_safety_gate_node", options), is_safe_(false), all_pins_initialized_(false) {
  // Declare parameters
  this->declare_parameter<std::vector<int64_t>>("required_high_pins", std::vector<int64_t>{});
  this->declare_parameter<std::vector<int64_t>>("required_low_pins", std::vector<int64_t>{});
  this->declare_parameter<std::vector<int64_t>>("any_high_pins", std::vector<int64_t>{});
  this->declare_parameter<double>("timeout_duration", 1.0);
  this->declare_parameter<std::string>("logic_mode", "AND");
  this->declare_parameter<bool>("default_safe_state", false);

  // Get parameters
  required_high_pins_ = this->get_parameter("required_high_pins").as_integer_array();
  required_low_pins_ = this->get_parameter("required_low_pins").as_integer_array();
  any_high_pins_ = this->get_parameter("any_high_pins").as_integer_array();
  timeout_duration_ = this->get_parameter("timeout_duration").as_double();
  logic_mode_ = this->get_parameter("logic_mode").as_string();
  default_safe_state_ = this->get_parameter("default_safe_state").as_bool();

  RCLCPP_INFO(this->get_logger(), "GPIO Safety Gate Component initialized");
  RCLCPP_INFO(this->get_logger(), "Logic mode: %s", logic_mode_.c_str());
  RCLCPP_INFO(this->get_logger(), "Timeout: %.2f seconds", timeout_duration_);
  RCLCPP_INFO(this->get_logger(), "Default safe state: %s", default_safe_state_ ? "true" : "false");

  // Log pin configurations
  if (!required_high_pins_.empty()) {
    std::string pins_str = "";
    for (const auto& pin : required_high_pins_) {
      pins_str += std::to_string(pin) + " ";
    }
    RCLCPP_INFO(this->get_logger(), "Required HIGH pins: %s", pins_str.c_str());
  }

  if (!required_low_pins_.empty()) {
    std::string pins_str = "";
    for (const auto& pin : required_low_pins_) {
      pins_str += std::to_string(pin) + " ";
    }
    RCLCPP_INFO(this->get_logger(), "Required LOW pins: %s", pins_str.c_str());
  }

  if (!any_high_pins_.empty()) {
    std::string pins_str = "";
    for (const auto& pin : any_high_pins_) {
      pins_str += std::to_string(pin) + " ";
    }
    RCLCPP_INFO(this->get_logger(), "Any HIGH pins: %s", pins_str.c_str());
  }

  // Initialize GPIO states
  for (const auto& pin : required_high_pins_) {
    gpio_states_[pin] = false;
  }
  for (const auto& pin : required_low_pins_) {
    gpio_states_[pin] = false;
  }
  for (const auto& pin : any_high_pins_) {
    gpio_states_[pin] = false;
  }

  // Create subscribers for all monitored GPIO pins
  std::vector<int64_t> all_pins;
  all_pins.insert(all_pins.end(), required_high_pins_.begin(), required_high_pins_.end());
  all_pins.insert(all_pins.end(), required_low_pins_.begin(), required_low_pins_.end());
  all_pins.insert(all_pins.end(), any_high_pins_.begin(), any_high_pins_.end());

  for (const auto& pin : all_pins) {
    std::string topic_name = "gpio_" + std::to_string(pin);
    gpio_subs_[pin] = this->create_subscription<std_msgs::msg::Bool>(
        topic_name, 10,
        [this, pin](const std_msgs::msg::Bool::SharedPtr msg) { this->gpio_callback(msg, pin); });
    RCLCPP_INFO(this->get_logger(), "Subscribed to topic: %s", topic_name.c_str());
  }

  // Create safety state publisher
  safety_state_pub_ = this->create_publisher<std_msgs::msg::Bool>("safety_state", 10);

  // Create timer for periodic safety check
  auto timer_interval = std::chrono::duration<double>(0.1);  // 10Hz
  check_timer_ = this->create_wall_timer(
      timer_interval, std::bind(&GpioSafetyGateComponent::check_safety_state, this));

  RCLCPP_INFO(this->get_logger(), "GPIO Safety Gate Component started");
}

GpioSafetyGateComponent::~GpioSafetyGateComponent() {
  RCLCPP_INFO(this->get_logger(), "GPIO Safety Gate Component destroyed");
}

void GpioSafetyGateComponent::gpio_callback(const std_msgs::msg::Bool::SharedPtr msg,
                                            unsigned int pin) {
  gpio_states_[pin] = msg->data;
  last_update_times_[pin] = std::chrono::steady_clock::now();

  // Check if all pins have been initialized
  if (!all_pins_initialized_) {
    bool all_updated = true;
    for (const auto& pair : gpio_states_) {
      if (last_update_times_.find(pair.first) == last_update_times_.end()) {
        all_updated = false;
        break;
      }
    }
    if (all_updated) {
      all_pins_initialized_ = true;
      RCLCPP_INFO(this->get_logger(), "All GPIO pins initialized");
    }
  }

  // Immediately check safety state on GPIO update
  check_safety_state();
}

bool GpioSafetyGateComponent::evaluate_safety_condition() {
  // If not all pins are initialized, return default safe state
  if (!all_pins_initialized_) {
    return default_safe_state_;
  }

  // Check for timeout
  auto now = std::chrono::steady_clock::now();
  for (const auto& pair : last_update_times_) {
    auto elapsed = std::chrono::duration<double>(now - pair.second).count();
    if (elapsed > timeout_duration_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "GPIO pin %u timeout (%.2f seconds)", pair.first, elapsed);
      return false;  // Not safe if any pin times out
    }
  }

  bool result = false;

  if (logic_mode_ == "AND") {
    // All required_high_pins must be HIGH
    // All required_low_pins must be LOW
    result = true;

    for (const auto& pin : required_high_pins_) {
      if (!gpio_states_[pin]) {
        result = false;
        break;
      }
    }

    if (result) {
      for (const auto& pin : required_low_pins_) {
        if (gpio_states_[pin]) {
          result = false;
          break;
        }
      }
    }

  } else if (logic_mode_ == "OR") {
    // At least one pin from any_high_pins must be HIGH
    result = false;
    for (const auto& pin : any_high_pins_) {
      if (gpio_states_[pin]) {
        result = true;
        break;
      }
    }

  } else if (logic_mode_ == "CUSTOM") {
    // Custom logic: Combination of AND and OR
    // All required pins must match AND at least one from any_high_pins
    bool required_check = true;

    for (const auto& pin : required_high_pins_) {
      if (!gpio_states_[pin]) {
        required_check = false;
        break;
      }
    }

    if (required_check) {
      for (const auto& pin : required_low_pins_) {
        if (gpio_states_[pin]) {
          required_check = false;
          break;
        }
      }
    }

    bool any_check = any_high_pins_.empty() ? true : false;
    for (const auto& pin : any_high_pins_) {
      if (gpio_states_[pin]) {
        any_check = true;
        break;
      }
    }

    result = required_check && any_check;

  } else {
    RCLCPP_ERROR(this->get_logger(), "Unknown logic mode: %s", logic_mode_.c_str());
    result = false;
  }

  return result;
}

void GpioSafetyGateComponent::check_safety_state() {
  bool new_safe_state = evaluate_safety_condition();

  // Publish safety state if changed
  if (new_safe_state != is_safe_) {
    is_safe_ = new_safe_state;

    if (is_safe_) {
      RCLCPP_INFO(this->get_logger(), "✓ System is SAFE - Operation enabled");
    } else {
      RCLCPP_WARN(this->get_logger(), "✗ System is UNSAFE - Operation disabled");
    }
  }

  // Always publish current state
  auto msg = std_msgs::msg::Bool();
  msg.data = is_safe_;
  safety_state_pub_->publish(msg);
}

}  // namespace gpio_safety_gate

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(gpio_safety_gate::GpioSafetyGateComponent)
