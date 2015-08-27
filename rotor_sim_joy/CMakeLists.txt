cmake_minimum_required(VERSION 2.8.3)
project(rotor_sim_joy)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  rotor_gazebo
  sensor_msgs
  trajectory_msgs
  relative_nav_msgs
)
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  DEPENDS
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs rotor_gazebo sensor_msgs trajectory_msgs relative_nav_msgs
  LIBRARIES
)

add_definitions(-std=c++11)

add_executable(rotor_sim_joy src/joy.cpp include/rotor_sim_joy/joy.h)
add_dependencies(rotor_sim_joy relative_nav_msgs_generate_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(rotor_sim_joy ${catkin_LIBRARIES})

install(TARGETS rotor_sim_joy
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
