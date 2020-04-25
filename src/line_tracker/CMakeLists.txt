cmake_minimum_required(VERSION 2.8.3)
project(line_tracker)
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  core_msgs
  image_transport
  message_generation
  roscpp
  sensor_msgs
  std_msgs
)

find_package( OpenCV REQUIRED )
find_package( cv_bridge REQUIRED )

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  #include
)

add_executable(line_detect_node src/line_detect.cpp)
add_dependencies(line_detect_node core_msgs_generate_messages_cpp)

target_link_libraries(line_detect_node
  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${cv_bridge_LIBRARIES}
)