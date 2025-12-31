cmake_minimum_required(VERSION 3.8)
project(click_path_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
#find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(spot_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_library(grid_core
  src/grid_map.cpp
  src/point_cloud_handler.cpp
  src/grid_interface.cpp
)

target_link_libraries(grid_core
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  Eigen3::Eigen
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
)

add_executable(click_path_planner
  src/click_path_planner.cpp
)

ament_target_dependencies(click_path_planner
  #ament_index_cpp
  rclcpp
  rclcpp_action
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  spot_action
)

target_link_libraries(click_path_planner grid_core)

install(TARGETS grid_core click_path_planner
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install data (PCD files)
install(
  DIRECTORY data
  DESTINATION share/${PROJECT_NAME}/
)

#install(
#  DIRECTORY saves
#  DESTINATION share/${PROJECT_NAME}/
#)

# Install your executable
install(
  TARGETS click_path_planner
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
