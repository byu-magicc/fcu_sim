cmake_minimum_required(VERSION 2.8.3)
project(sim_reset)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  gazebo_msgs
  roscpp
  rospy
  relative_nav
)

catkin_package()

###########
## Build ##
###########
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(sim_reset
#   src/${PROJECT_NAME}/sim_reset.cpp
# )

## Declare a cpp executable
add_executable(sim_reset_node src/sim_reset_node.cpp include/sim_reset/sim_reset.h src/sim_reset.cpp)

## Add cmake target dependencies of the executable/library
add_dependencies(sim_reset_node relative_nav_generate_messages_cpp)

target_link_libraries(sim_reset_node
  ${catkin_LIBRARIES}
)
