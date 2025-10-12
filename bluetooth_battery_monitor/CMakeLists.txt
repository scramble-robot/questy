cmake_minimum_required(VERSION 3.8)
project(bluetooth_battery_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories
include_directories(include)

# Component library
add_library(${PROJECT_NAME}_component SHARED
  src/bluetooth_battery_monitor_node.cpp
)

target_compile_features(${PROJECT_NAME}_component PUBLIC cxx_std_17)

ament_target_dependencies(${PROJECT_NAME}_component
  rclcpp
  rclcpp_components
  sensor_msgs
  diagnostic_updater
  std_msgs
)

# Register component
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "bluetooth_battery_monitor::BluetoothBatteryMonitor"
  EXECUTABLE bluetooth_battery_monitor_node
)

# Install component library
install(TARGETS ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
