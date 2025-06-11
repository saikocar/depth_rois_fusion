cmake_minimum_required(VERSION 3.5)
project(depth_rois_fusion)

# C++14 or later
set(CMAKE_CXX_STANDARD 14)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_geometry REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tier4_perception_msgs REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(depth_rois_fusion_node src/depth_rois_fusion.cpp)

ament_target_dependencies(depth_rois_fusion_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  cv_bridge
  OpenCV
  image_geometry
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  Eigen3
  tier4_perception_msgs
)

target_link_libraries(depth_rois_fusion_node ${OpenCV_LIBS})

install(TARGETS depth_rois_fusion_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
install(FILES package.xml DESTINATION share/${PROJECT_NAME})
ament_package()

