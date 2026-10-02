cmake_minimum_required(VERSION 3.8)
project(target_ekf_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Search System and ROS2 Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# 2. Declare C++ Executable
add_executable(ekf_node src/ekf_node.cpp)

# 3. Include Heading Routes in Eigen
target_include_directories(ekf_node PUBLIC ${EIGEN3_INCLUDE_DIRS})

# 4. Link ROS2 Dependencies with Executable
ament_target_dependencies(
  ekf_node
  rclcpp
  geometry_msgs
  nav_msgs
  visualization_msgs
)

# 5. Install Executable
install(TARGETS
  ekf_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()