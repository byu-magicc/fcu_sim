cmake_minimum_required(VERSION 2.8.3)
project(rotors_gazebo_plugins)

add_definitions(-std=c++11)

# To enable assertions when compiled in release mode.
add_definitions(-DROS_ASSERT_ENABLED)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  cv_bridge
  geometry_msgs
  mav_msgs
  octomap_msgs
  octomap_ros
  rosbag
  roscpp
  rotors_comm
  rotors_control
  std_srvs
  tf
  relative_nav_msgs
  relative_nav_common
)

find_package(Eigen REQUIRED)
find_package(octomap REQUIRED)
find_package(gazebo REQUIRED)
find_package(OpenCV REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  LIBRARIES rotors_gazebo_motor_model rotors_gazebo_controller_interface
  CATKIN_DEPENDS cv_bridge geometry_msgs mav_msgs octomap_msgs octomap_ros rosbag roscpp 
                 rotors_comm rotors_control std_srvs tf relative_nav_msgs relative_nav_common
  DEPENDS eigen gazebo octomap opencv
)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})

add_library(rotors_gazebo_motor_model 
  src/gazebo_motor_model.cpp  
  include/rotors_gazebo_plugins/motor_model.hpp
  include/rotors_gazebo_plugins/gazebo_motor_model.h
  include/rotors_gazebo_plugins/common.h   
  include/rotors_gazebo_plugins/motor_controller.hpp  
  include/rotors_gazebo_plugins/multi_copter.hpp)
target_link_libraries(rotors_gazebo_motor_model ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(rotors_gazebo_motor_model ${catkin_EXPORTED_TARGETS})

add_library(rotors_gazebo_odometry_plugin 
  src/gazebo_odometry_plugin.cpp
  include/rotors_gazebo_plugins/gazebo_odometry_plugin.h)
target_link_libraries(rotors_gazebo_odometry_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(rotors_gazebo_odometry_plugin ${catkin_EXPORTED_TARGETS})

add_library(rotors_gazebo_imu_plugin 
  src/gazebo_imu_plugin.cpp
  include/rotors_gazebo_plugins/gazebo_imu_plugin.h)
target_link_libraries(rotors_gazebo_imu_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(rotors_gazebo_imu_plugin ${catkin_EXPORTED_TARGETS})

add_library(rotors_gazebo_altimeter_plugin 
  src/gazebo_altimeter_plugin.cpp
  include/rotors_gazebo_plugins/gazebo_altimeter_plugin.h)
target_link_libraries(rotors_gazebo_altimeter_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(rotors_gazebo_altimeter_plugin ${catkin_EXPORTED_TARGETS})

add_library(rotors_gazebo_global_state_plugin 
  src/gazebo_global_state_plugin.cpp
  include/rotors_gazebo_plugins/gazebo_global_state_plugin.h)
target_link_libraries(rotors_gazebo_global_state_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(rotors_gazebo_global_state_plugin ${catkin_EXPORTED_TARGETS})

add_library(rotors_gazebo_controller_interface 
  src/gazebo_controller_interface.cpp
  include/rotors_gazebo_plugins/gazebo_controller_interface.h)
target_link_libraries(rotors_gazebo_controller_interface ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(rotors_gazebo_controller_interface ${catkin_EXPORTED_TARGETS})

add_library(rotors_gazebo_bag_plugin 
  src/gazebo_bag_plugin.cpp
  include/rotors_gazebo_plugins/gazebo_bag_plugin.h)
target_link_libraries(rotors_gazebo_bag_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(rotors_gazebo_bag_plugin ${catkin_EXPORTED_TARGETS})

add_library(rotors_gazebo_wind_plugin 
  src/gazebo_wind_plugin.cpp
  include/rotors_gazebo_plugins/gazebo_wind_plugin.h)
target_link_libraries(rotors_gazebo_wind_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(rotors_gazebo_wind_plugin ${catkin_EXPORTED_TARGETS})

add_library(rotors_gazebo_octomap_plugin 
  src/gazebo_octomap_plugin.cpp
  include/rotors_gazebo_plugins/gazebo_octomap_plugin.h)
target_link_libraries(rotors_gazebo_octomap_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(rotors_gazebo_octomap_plugin ${catkin_EXPORTED_TARGETS})

add_library(rotors_gazebo_multirotor_base_plugin 
  src/gazebo_multirotor_base_plugin.cpp
  include/rotors_gazebo_plugins/gazebo_multirotor_base_plugin.h)
target_link_libraries(rotors_gazebo_multirotor_base_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(rotors_gazebo_multirotor_base_plugin ${catkin_EXPORTED_TARGETS})

install(
  TARGETS
    rotors_gazebo_motor_model
    rotors_gazebo_odometry_plugin
    rotors_gazebo_imu_plugin
    rotors_gazebo_altimeter_plugin
    rotors_gazebo_controller_interface
    rotors_gazebo_bag_plugin
    rotors_gazebo_wind_plugin
    rotors_gazebo_octomap_plugin
    rotors_gazebo_multirotor_base_plugin
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
