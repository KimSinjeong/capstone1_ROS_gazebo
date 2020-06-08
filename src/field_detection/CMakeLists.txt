cmake_minimum_required(VERSION 2.8.3)
project(field_detection)
add_compile_options(-std=c++11)

## Compile as C++11, supported in ROS Kinetic and newer

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  core_msgs
  pcl_ros
  pcl_conversions
  image_transport
  cv_bridge
)

find_package( OpenCV REQUIRED )

catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS roscpp
#  sensor_msgs
#  std_msgs
#  core_msgs
#  image_transport
#  DEPENDS system_lib
)
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  #include
)
add_executable(ball_detect_node src/ball_detect.cpp)
add_dependencies(ball_detect_node core_msgs_generate_messages_cpp)

target_link_libraries(ball_detect_node
  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${cv_bridge_LIBRARIES}
)

add_executable(dead_reckoning_node src/dead_reckoning.cpp)
add_dependencies(dead_reckoning_node core_msgs_generate_messages_cpp)

target_link_libraries(dead_reckoning_node
  ${catkin_LIBRARIES} ${OpenCV_LIBS}
)