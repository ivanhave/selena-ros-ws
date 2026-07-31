cmake_minimum_required(VERSION 3.8)
project(robot_gantry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(urdf REQUIRED)

# ── Interfaces ────────────────────────────────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GantryMove.action"
  "srv/GantrySetMode.srv"
  DEPENDENCIES std_msgs
)

# ── Header-only constants library — exported for downstream consumers ─────────
add_library(gantry_constants INTERFACE)
target_include_directories(gantry_constants INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ── Node executable ───────────────────────────────────────────────────────────
add_executable(gantry_node src/gantry_node.cpp)

ament_target_dependencies(gantry_node
  rclcpp
  rclcpp_action
  control_msgs
  controller_manager_msgs
  sensor_msgs
  std_msgs
  trajectory_msgs
  urdf
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(gantry_node "${cpp_typesupport_target}" gantry_constants)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS gantry_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS gantry_constants
  EXPORT gantry_constantsTargets
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(gantry_constantsTargets)
ament_export_include_directories(include)

ament_package()
