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
  octomap_msgs
  std_msgs
)

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

# Declare the service files to be built
add_service_files(
  FILES
  Octomap.srv
)

add_message_files(
  FILES
  WindSpeed.msg
  Actuators.msg
  AttitudeThrust.msg
  RateThrust.msg
  RollPitchYawrateThrust.msg
  TorqueThrust.msg
  Status.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs octomap_msgs)

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

# Build
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})

add_executable(waypoint_publisher src/waypoint_publisher.cpp)
target_link_libraries(waypoint_publisher ${catkin_LIBRARIES})
add_dependencies(waypoint_publisher rotor_gazebo_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(waypoint_publisher_file src/waypoint_publisher_file.cpp)
target_link_libraries(waypoint_publisher_file ${catkin_LIBRARIES})
add_dependencies(waypoint_publisher_file rotor_gazebo_generate_messages_cpp  ${catkin_EXPORTED_TARGETS})

add_executable(hovering_example src/hovering_example.cpp)
target_link_libraries(hovering_example ${catkin_LIBRARIES})
add_dependencies(hovering_example rotor_gazebo_generate_messages_cpp  ${catkin_EXPORTED_TARGETS})

foreach(dir launch models resource worlds)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

install(TARGETS waypoint_publisher waypoint_publisher_file hovering_example
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
