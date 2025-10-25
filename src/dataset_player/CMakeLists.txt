cmake_minimum_required(VERSION 3.8)
project(dataset_player)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(player src/player.cpp)
ament_target_dependencies(player rclcpp sensor_msgs)

install(TARGETS player
  DESTINATION lib/${PROJECT_NAME})

ament_package()
