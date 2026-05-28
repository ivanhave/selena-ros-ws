cmake_minimum_required(VERSION 3.8)
project(robot_gripper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

# ── Driver library ───────────────────────────────────────
add_library(mg996r_driver src/mg996r_driver.cpp)
target_include_directories(mg996r_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mg996r_driver rclcpp)

# ── Gripper interface node ───────────────────────────────
add_executable(gripper_interface_node src/gripper_interface.cpp)
target_include_directories(gripper_interface_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(gripper_interface_node mg996r_driver)
ament_target_dependencies(gripper_interface_node rclcpp std_msgs)

# ── Gripper simulator (no CAN — for offline testing) ────
add_executable(gripper_sim_node src/gripper_sim_node.cpp)
target_include_directories(gripper_sim_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(gripper_sim_node rclcpp std_msgs)

# ── Test executable ──────────────────────────────────────
add_executable(gripper_test test/gripper_test.cpp)
target_include_directories(gripper_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(gripper_test mg996r_driver)
ament_target_dependencies(gripper_test rclcpp)

# ── Install ──────────────────────────────────────────────
install(TARGETS mg996r_driver gripper_interface_node gripper_sim_node gripper_test
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()