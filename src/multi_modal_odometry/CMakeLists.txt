cmake_minimum_required(VERSION 3.8)
project(multi_modal_odometry)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration)
find_package(gtsam REQUIRED)

add_definitions(${PCL_DEFINITIONS})

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${gtsam_INCLUDE_DIR}
)

add_executable(vio_node src/vio_node.cpp)
target_link_libraries(vio_node
  ${OpenCV_LIBS}
  Eigen3::Eigen
)
ament_target_dependencies(vio_node
  rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_eigen tf2_geometry_msgs cv_bridge
)

add_executable(lidar_odom_node src/lidar_odom_node.cpp)
target_link_libraries(lidar_odom_node
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)
ament_target_dependencies(lidar_odom_node
  rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_eigen tf2_geometry_msgs
)

add_executable(fusion_node src/fusion_node.cpp)
target_link_libraries(fusion_node
  Eigen3::Eigen
  gtsam
)
ament_target_dependencies(fusion_node
  rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_eigen tf2_geometry_msgs
)

install(TARGETS vio_node lidar_odom_node fusion_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
