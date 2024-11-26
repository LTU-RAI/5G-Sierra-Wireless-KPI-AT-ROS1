cmake_minimum_required(VERSION 3.5)
project(nr5g_sierra_at)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Add custom message files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/at_sierra_nr5g.msg"
  DEPENDENCIES std_msgs
)

# Install Python node
install(PROGRAMS
  src/nr5gsierra.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config files (if any)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
