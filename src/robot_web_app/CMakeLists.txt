cmake_minimum_required(VERSION 3.8)
project(robot_web_app)

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install webapp files
install(DIRECTORY webapp/
  DESTINATION share/${PROJECT_NAME}/webapp
)

# Install scripts as executables
install(PROGRAMS scripts/serve_webapp.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()