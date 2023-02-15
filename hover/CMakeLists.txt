cmake_minimum_required(VERSION 3.0.2)
project(hover)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  roslaunch
  rospy
  std_msgs
  nav_msgs
  mav_trajectory_generation 
  mav_trajectory_generation_ros
  message_generation
)

add_message_files(
  FILES
  FlatTarget.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES hover
   CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp rospy std_msgs nav_msgs mav_trajectory_generation mav_trajectory_generation_ros message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

roslaunch_add_file_check(launch)

add_library(hover_lib
  src/hover_lib.cpp
)
target_link_libraries(hover_lib
  ${catkin_LIBRARIES}
)

add_executable(hover_node src/hover_node.cpp)
target_link_libraries(hover_node
  hover_lib
)

add_executable(setmode_hover src/setmode_hover.cpp)
target_link_libraries(setmode_hover
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/MarkerDetection.py
  scripts/real_cam.py
  scripts/transform.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
