cmake_minimum_required(VERSION 2.8.3)
project(attitude_controller)

find_package(catkin REQUIRED COMPONENTS
  rotor_gazebo
  nav_msgs
  roscpp
  relative_nav
  tf
  geometry_msgs
  cmake_modules
)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  CATKIN_DEPENDS geometry_msgs rotor_gazebo nav_msgs roscpp
  DEPENDS Eigen
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(attitude_controller
#   src/${PROJECT_NAME}/attitude_controller.cpp
# )

## Declare a cpp executable
add_executable(attitude_controller_node
                src/attitude_controller_node.cpp
                src/attitude_controller.cpp
                include/attitude_controller/attitude_controller.h
                include/attitude_controller/multicopter.h
                src/h_inf_controller.cpp
                include/attitude_controller/h_inf_controller.h
)

## Add cmake target dependencies of the executable/library
add_dependencies(attitude_controller_node relative_nav_generate_messages_cpp)

target_link_libraries(attitude_controller_node
  ${catkin_LIBRARIES}
)
