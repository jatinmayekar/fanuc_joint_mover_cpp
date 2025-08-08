cmake_minimum_required(VERSION 3.8)
project(joint_mover_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies - now includes fanuc_msgs
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(fanuc_msgs REQUIRED)

# Add executable
add_executable(j2_mover_cpp src/j2_mover.cpp)
ament_target_dependencies(j2_mover_cpp 
  rclcpp 
  trajectory_msgs 
  sensor_msgs
  fanuc_msgs
)

# Install executable
install(TARGETS
  j2_mover_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()