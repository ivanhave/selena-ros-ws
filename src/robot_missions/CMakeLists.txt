cmake_minimum_required(VERSION 3.8)
project(robot_missions)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ── Action interfaces ──────────────────────────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MissionAction.action"
  DEPENDENCIES std_msgs
)

# ── Library — all mission code ─────────────────────────────────────────────────
add_library(${PROJECT_NAME}_lib
  src/mission_registry.cpp
  src/mission_manager.cpp
  src/planting/planting_mission.cpp
  src/planting/seed_database.cpp
  src/planting/planting_planner.cpp
  src/planting/gripper_tool.cpp
  src/planting/execution_engine.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_lib
  ament_index_cpp
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  trajectory_msgs
)

# Link generated action typesupport into the library
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(${PROJECT_NAME}_lib ${cpp_typesupport_target})

# ── Executable — mission action server ────────────────────────────────────────
add_executable(mission_server src/mission_server_node.cpp)

target_include_directories(mission_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(mission_server ${PROJECT_NAME}_lib)

ament_target_dependencies(mission_server
  ament_index_cpp
  rclcpp
  rclcpp_action
)

# ── Install ────────────────────────────────────────────────────────────────────
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}_lib
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS mission_server
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  FILES ARCHITECTURE.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
