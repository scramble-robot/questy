cmake_minimum_required(VERSION 3.5)
project(operation_manager)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(operation_manager_component SHARED
  src/operation_manager_component.cpp
)

rclcpp_components_register_nodes(operation_manager_component
  "operation_manager::OperationManagerComponent")

ament_auto_add_executable(operation_manager_node
  src/operation_manager_node.cpp
)

target_link_libraries(operation_manager_node operation_manager_component)

ament_auto_package(INSTALL_TO_SHARE launch config)
