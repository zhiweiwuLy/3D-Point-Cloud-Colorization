cmake_minimum_required(VERSION 3.0.2)
project(rgb_pointcloud)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  cv_bridge
  image_transport
  sensor_msgs
  std_msgs
)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenMP REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)
catkin_package()
add_executable(rgb_mapper_node src/rgb_mapper_node.cpp)
target_link_libraries(rgb_mapper_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
)