cmake_minimum_required(VERSION 3.8)
project(gpio_reader)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
)

# Create the component library
add_library(gpio_reader_component SHARED
  src/gpio_reader_component.cpp
)

target_compile_definitions(gpio_reader_component
  PRIVATE "GPIO_READER_BUILDING_DLL")

ament_target_dependencies(gpio_reader_component
  rclcpp
  rclcpp_components
  std_msgs
)

# Link with gpiod library for Raspberry Pi 5
target_link_libraries(gpio_reader_component gpiod)

# Register the component
rclcpp_components_register_nodes(gpio_reader_component
  "gpio_reader::GpioReaderComponent")

# Create the executable
add_executable(gpio_reader_node
  src/gpio_reader_node.cpp
)

ament_target_dependencies(gpio_reader_node
  rclcpp
  rclcpp_components
)

target_link_libraries(gpio_reader_node gpio_reader_component)

# Install targets
install(TARGETS
  gpio_reader_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  gpio_reader_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(gpio_reader_component)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  std_msgs
)

ament_package()
