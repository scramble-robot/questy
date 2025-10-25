#include <memory>

#include "gpio_safety_gate/gpio_safety_gate_component.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto node = std::make_shared<gpio_safety_gate::GpioSafetyGateComponent>(options);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
