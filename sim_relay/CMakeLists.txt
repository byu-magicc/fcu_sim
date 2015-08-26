cmake_minimum_required(VERSION 2.8.3)
project(sim_relay)

find_package(catkin REQUIRED COMPONENTS
  relative_nav_msgs
  relative_nav_msgs
  roscpp
  rospy
  nav_msgs
  geometry_msgs
  sensor_msgs
  mav_msgs
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
# add_library(sim_relay
#   src/${PROJECT_NAME}/sim_relay.cpp
# )

## Declare a cpp executable
add_executable(sim_relay_node src/sim_relay_node.cpp src/sim_relay.cpp include/sim_relay/sim_relay.h)

## Add cmake target dependencies of the executable/library
add_dependencies(sim_relay_node
                relative_nav_msgs_generate_messages_cpp
                mav_msgs_generate_msgs_cpp
)

target_link_libraries(sim_relay_node
  ${catkin_LIBRARIES}
)
