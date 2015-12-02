cmake_minimum_required(VERSION 2.8.3)
project(joy_sim)

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

add_executable(joy_sim src/joy.cpp include/joy_sim/joy.h)
add_dependencies(joy_sim relative_nav_msgs_generate_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(joy_sim ${catkin_LIBRARIES})

install(TARGETS joy_sim
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
