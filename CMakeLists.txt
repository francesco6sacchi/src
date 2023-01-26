cmake_minimum_required(VERSION 2.8.3)
project(safety_index)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  message_generation
  dsr_msgs
)

add_service_files(
  FILES
  compute.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ~/libraries/eigen-3.4.0
  ~/libraries
  ${catkin_INCLUDE_DIRS}
)

add_executable(robot_node src/robot_node.cpp src/robot.cpp src/geometry.cpp src/QuickHull.cpp)
add_executable(robot_old src/robot_old_node.cpp src/robot_old.cpp src/geometry.cpp src/QuickHull.cpp)
target_link_libraries(robot_node ${catkin_LIBRARIES})
target_link_libraries(robot_old ${catkin_LIBRARIES})