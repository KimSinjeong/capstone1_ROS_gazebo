cmake_minimum_required(VERSION 2.8.3)
project(line_tracing)
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
#  visualization_msgs
#  DEPENDS system_lib
)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  #include
)

add_executable(line_detect_node src/line_detect.cpp)
# add_dependencies(line_detect_node core_msgs_generate_messages_cpp)

target_link_libraries(line_detect_node
  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${cv_bridge_LIBRARIES}
)


add_executable(line_detect_actuation src/line_detect_actuation.cpp)
# add_dependencies(line_detect_actuation core_msgs_generate_messages_cpp)

target_link_libraries(line_detect_actuation
  ${catkin_LIBRARIES} ${cv_bridge_LIBRARIES}
)
