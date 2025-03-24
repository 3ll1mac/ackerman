cmake_minimum_required(VERSION 3.8)
project(ackerman_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
#find_package(gazebo_ros REQUIRED)

add_executable(example_ackermann_drive examples/example_ackermann_drive.cpp)
ament_target_dependencies(example_ackermann_drive
  rclcpp
  geometry_msgs
)


install(
  DIRECTORY launch model worlds urdf config
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
    example_ackermann_drive
  DESTINATION
    lib/${PROJECT_NAME}
)


ament_package()
