cmake_minimum_required(VERSION 2.8.3)
project(rotor_gazebo_plugins)

add_definitions(-std=c++11)

# To enable assertions when compiled in release mode.
add_definitions(-DROS_ASSERT_ENABLED)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  cv_bridge
  geometry_msgs
  rotor_gazebo
  rosbag
  roscpp
  rotor_gazebo
  std_srvs
  tf
  relative_nav_msgs
  relative_nav_common
)

find_package(Eigen REQUIRED)
find_package(gazebo REQUIRED)
find_package(OpenCV REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  LIBRARIES rotor_gazebo_motor_model rotor_gazebo_controller_interface
  CATKIN_DEPENDS cv_bridge geometry_msgs rotor_gazebo rosbag roscpp
                 rotor_gazebo std_srvs tf relative_nav_msgs relative_nav_common
  DEPENDS eigen gazebo opencv #octomap
)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})

add_library(rotor_gazebo_motor_model 
  src/gazebo_motor_model.cpp  
  include/rotor_gazebo_plugins/motor_model.hpp
  include/rotor_gazebo_plugins/gazebo_motor_model.h
  include/rotor_gazebo_plugins/common.h   
  include/rotor_gazebo_plugins/motor_controller.hpp  
  include/rotor_gazebo_plugins/multi_copter.hpp)
target_link_libraries(rotor_gazebo_motor_model ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(rotor_gazebo_motor_model ${catkin_EXPORTED_TARGETS})

add_library(rotor_gazebo_aircraft_forces_and_moments_plugin
  src/gazebo_aircraft_forces_and_moments.cpp
  include/rotor_gazebo_plugins/gazebo_aircraft_forces_and_moments.h)
target_link_libraries(rotor_gazebo_aircraft_forces_and_moments_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(rotor_gazebo_aircraft_forces_and_moments_plugin ${catkin_EXPORTED_TARGETS})

add_library(rotor_gazebo_odometry_plugin 
  src/gazebo_odometry_plugin.cpp
  include/rotor_gazebo_plugins/gazebo_odometry_plugin.h)
target_link_libraries(rotor_gazebo_odometry_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(rotor_gazebo_odometry_plugin ${catkin_EXPORTED_TARGETS})

add_library(rotor_gazebo_imu_plugin 
  src/gazebo_imu_plugin.cpp
  include/rotor_gazebo_plugins/gazebo_imu_plugin.h)
target_link_libraries(rotor_gazebo_imu_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(rotor_gazebo_imu_plugin ${catkin_EXPORTED_TARGETS})

add_library(rotor_gazebo_altimeter_plugin 
  src/gazebo_altimeter_plugin.cpp
  include/rotor_gazebo_plugins/gazebo_altimeter_plugin.h)
target_link_libraries(rotor_gazebo_altimeter_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(rotor_gazebo_altimeter_plugin ${catkin_EXPORTED_TARGETS})

add_library(rotor_gazebo_wind_plugin 
  src/gazebo_wind_plugin.cpp
  include/rotor_gazebo_plugins/gazebo_wind_plugin.h)
target_link_libraries(rotor_gazebo_wind_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(rotor_gazebo_wind_plugin ${catkin_EXPORTED_TARGETS})

add_library(rotor_gazebo_multirotor_base_plugin 
  src/gazebo_multirotor_base_plugin.cpp
  include/rotor_gazebo_plugins/gazebo_multirotor_base_plugin.h)
target_link_libraries(rotor_gazebo_multirotor_base_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(rotor_gazebo_multirotor_base_plugin ${catkin_EXPORTED_TARGETS})

add_library(rotor_gazebo_controller_interface 
  src/gazebo_controller_interface.cpp
  include/rotor_gazebo_plugins/gazebo_controller_interface.h)
target_link_libraries(rotor_gazebo_controller_interface ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(rotor_gazebo_controller_interface ${catkin_EXPORTED_TARGETS})

install(
  TARGETS
    rotor_gazebo_motor_model
    rotor_gazebo_odometry_plugin
    rotor_gazebo_imu_plugin
    rotor_gazebo_altimeter_plugin
    rotor_gazebo_wind_plugin
    rotor_gazebo_multirotor_base_plugin
    rotor_gazebo_controller_interface 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
