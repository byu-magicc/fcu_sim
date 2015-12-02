cmake_minimum_required(VERSION 2.8.3)
project(rotor_gazebo)

add_definitions(-std=c++11)

find_package(catkin 
  REQUIRED COMPONENTS 
  gazebo_msgs 
  geometry_msgs 
  roscpp 
  sensor_msgs
  message_generation
  std_msgs
)

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

add_message_files(
  FILES
  WindSpeed.msg
  Actuators.msg
  AttitudeThrust.msg
  RateThrust.msg
  RollPitchYawrateThrust.msg
  TorqueThrust.msg
  Status.msg
  FWCommand.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

# Configure Build
catkin_package(
  INCLUDE_DIRS include ${EIGEN_INCLUDE_DIRS}
  CATKIN_DEPENDS
    gazebo_msgs
    geometry_msgs
    roscpp
    sensor_msgs
    message_runtime
  DEPENDS Eigen
  CFG_EXTRAS export_flags.cmake
)
