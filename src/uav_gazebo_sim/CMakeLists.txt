cmake_minimum_required(VERSION 3.8)
project(uav_gazebo_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)

# Install Python guidance script as ROS 2 executable
install(PROGRAMS
  uav_gazebo_sim/sting_guidance_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sting_guidance_controller
)

# Install directories to share folder
install(DIRECTORY
  launch
  models
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()