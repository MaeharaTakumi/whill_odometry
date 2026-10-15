cmake_minimum_required(VERSION 3.8)
project(whill_odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(whill_odometry_node src/odometry.cpp)
ament_target_dependencies(whill_odometry_node rclcpp geometry_msgs nav_msgs)

install(TARGETS whill_odometry_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY param launch DESTINATION share/${PROJECT_NAME})

ament_package()
