#include <rclcpp/rclcpp.hpp>

#include "operation_manager/operation_manager_component.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<operation_manager::OperationManagerComponent>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
