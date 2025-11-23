cmake_minimum_required(VERSION 3.8)
project(dataset_player)

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(player src/player.cpp)
target_compile_features(player PUBLIC cxx_std_17)
ament_target_dependencies(player rclcpp sensor_msgs cv_bridge image_transport)
target_link_libraries(player ${OpenCV_LIBRARIES})

install(TARGETS player
  DESTINATION lib/${PROJECT_NAME})

ament_package()
