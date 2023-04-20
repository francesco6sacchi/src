cmake_minimum_required(VERSION 2.8.3)
project(robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  message_generation
  dsr_msgs

  cv_bridge

  tf2
  tf2_ros
  
  interactive_markers
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)

include_directories(
  include
  /usr/include/opencv4
  /usr/local/include/eigen3
  # ~/libraries
  ${catkin_INCLUDE_DIRS}

)

# add_executable(robot_node src/robot_node.cpp src/robot.cpp src/geometry.cpp )
# target_link_libraries(robot_node ${catkin_LIBRARIES})
# add_executable(safety_index_node src/safety_index_node.cpp src/safety_index.cpp src/robot.cpp src/geometry.cpp )
# target_link_libraries(safety_index_node ${catkin_LIBRARIES})

add_executable(grasp_planning_node 
  src/Smartkit.cpp
  src/grasp_planning_node.cpp
  src/grasp_planning.cpp
  src/robot.cpp
  src/geometry.cpp
  src/object.cpp
  src/camera.cpp
  src/jsoncpp.cpp)

target_link_libraries(grasp_planning_node ${catkin_LIBRARIES})