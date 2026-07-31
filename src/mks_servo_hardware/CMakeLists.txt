cmake_minimum_required(VERSION 3.8)
project(mks_servo_hardware)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(robot_gantry REQUIRED)

add_library(${PROJECT_NAME} SHARED
    src/MKS_Driver.cpp
    src/gantry_hardware_interface.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
    hardware_interface rclcpp rclcpp_lifecycle pluginlib
)
target_link_libraries(${PROJECT_NAME} robot_gantry::gantry_constants)

add_executable(mks_driver_test test/mks_driver_test.cpp)
target_link_libraries(mks_driver_test ${PROJECT_NAME})
target_include_directories(mks_driver_test PRIVATE include)

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
install(TARGETS mks_driver_test
    DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
    DESTINATION include
)

pluginlib_export_plugin_description_file(hardware_interface mks_servo_hardware.xml)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(hardware_interface rclcpp rclcpp_lifecycle pluginlib)

ament_package()
