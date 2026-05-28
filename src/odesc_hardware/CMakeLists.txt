cmake_minimum_required(VERSION 3.8)
project(odesc_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

# ──────────────────────────────────────────
# Dependencies
# ──────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)

# ──────────────────────────────────────────
# Shared library — the ros2_control plugin
# ──────────────────────────────────────────
add_library(odesc_hardware SHARED
  src/can_interface.cpp
  src/odesc_driver.cpp
  src/odesc_hardware_interface.cpp
)

target_include_directories(odesc_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(odesc_hardware
  rclcpp
  hardware_interface
  pluginlib
  std_msgs
)

# ──────────────────────────────────────────
# Pluginlib XML
# ──────────────────────────────────────────
pluginlib_export_plugin_description_file(
  hardware_interface odesc_hardware.xml)

# ──────────────────────────────────────────
# Install
# ──────────────────────────────────────────
install(TARGETS odesc_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES odesc_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

install(FILES config/odesc_hardware.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_include_directories(include)
ament_export_libraries(odesc_hardware)
ament_export_dependencies(
  rclcpp
  hardware_interface
  pluginlib
)

# ──────────────────────────────────────────
# Standalone CAN test (no ROS runtime needed)
# ──────────────────────────────────────────
add_executable(odesc_test
  test/odesc_test.cpp
  src/can_interface.cpp
  src/odesc_driver.cpp
)

target_include_directories(odesc_test PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(odesc_test
  rclcpp
)

install(TARGETS odesc_test
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()