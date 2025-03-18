cmake_minimum_required(VERSION 3.8)
project(arm_srvs)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PickAndDropObject.srv"
  DEPENDENCIES geometry_msgs
)

# Install Python nodes
install(
  PROGRAMS
    arm_srvs/pick_service.py
    arm_srvs/drop_service.py
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

# Build and install
ament_package()