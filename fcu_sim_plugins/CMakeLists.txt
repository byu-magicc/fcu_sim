cmake_minimum_required(VERSION 2.8.3)
project(fcu_sim_plugins)

add_definitions(-std=c++11)

# To enable assertions when compiled in release mode.
add_definitions(-DROS_ASSERT_ENABLED)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  cv_bridge
  geometry_msgs
  fcu_sim
  rosbag
  roscpp
  std_srvs
  tf
  fcu_common
  sensor_msgs
)

find_package(Eigen3 REQUIRED)
find_package(gazebo REQUIRED)
find_package(OpenCV REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(lib/ROSflight_SIL)

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  CATKIN_DEPENDS cv_bridge geometry_msgs fcu_sim rosbag roscpp
                 fcu_sim std_srvs tf relative_nav relative_nav
  DEPENDS eigen gazebo opencv fcu_common
)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})

add_library(ROSflight_sil_plugin
 src/ROSflight_sil.cpp
 include/fcu_sim_plugins/ROSflight_sil.h

 lib/ROSflight/src/estimator.c
 lib/ROSflight/src/mixer.c
 lib/ROSflight/src/controller.c
 lib/ROSflight/src/sensors.c
 lib/ROSflight/src/mode.c
 lib/ROSflight/src/mux.c
 lib/ROSflight/lib/turbotrig/turbotrig.c
 lib/ROSflight/lib/turbotrig/turbovec.c
 lib/ROSflight_SIL/ROSflight_SIL.c)
target_link_libraries(ROSflight_sil_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(ROSflight_sil_plugin ${catkin_EXPORTED_TARGETS})
#set_property( TARGET ROSflight_sil_plugin APPEND_STRING PROPERTY COMPILE_FLAGS -w )

add_library(aircraft_forces_and_moments_plugin
  src/aircraft_forces_and_moments.cpp
  include/fcu_sim_plugins/aircraft_forces_and_moments.h)
target_link_libraries(aircraft_forces_and_moments_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(aircraft_forces_and_moments_plugin ${catkin_EXPORTED_TARGETS})

add_library(multirotor_forces_and_moments_plugin
  src/multirotor_forces_and_moments.cpp
  include/fcu_sim_plugins/multirotor_forces_and_moments.h)
target_link_libraries(multirotor_forces_and_moments_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(multirotor_forces_and_moments_plugin ${catkin_EXPORTED_TARGETS})

add_library(magnetometer_plugin
  src/magnetometer.cpp
include/fcu_sim_plugins/magnetometer.h)
target_link_libraries(magnetometer_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(magnetometer_plugin ${catkin_EXPORTED_TARGETS})

add_library(aircraft_truth_plugin
  src/aircraft_truth.cpp
  include/fcu_sim_plugins/aircraft_truth.h)
target_link_libraries(aircraft_truth_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(aircraft_truth_plugin ${catkin_EXPORTED_TARGETS})

add_library(odometry_plugin
  src/odometry_plugin.cpp
  include/fcu_sim_plugins/odometry_plugin.h)
target_link_libraries(odometry_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(odometry_plugin ${catkin_EXPORTED_TARGETS})

add_library(imu_plugin
  src/imu_plugin.cpp
  include/fcu_sim_plugins/imu_plugin.h)
target_link_libraries(imu_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(imu_plugin ${catkin_EXPORTED_TARGETS})

add_library(altimeter_plugin
  src/altimeter_plugin.cpp
  include/fcu_sim_plugins/altimeter_plugin.h)
target_link_libraries(altimeter_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(altimeter_plugin ${catkin_EXPORTED_TARGETS})

add_library(wind_plugin
  src/wind_plugin.cpp
  include/fcu_sim_plugins/wind_plugin.h)
target_link_libraries(wind_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(wind_plugin ${catkin_EXPORTED_TARGETS})

add_library(airspeed_plugin
  src/airspeed_plugin.cpp
  include/fcu_sim_plugins/airspeed_plugin.h)
target_link_libraries(airspeed_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(airspeed_plugin ${catkin_EXPORTED_TARGETS})

add_library(GPS_plugin
  src/GPS_plugin.cpp
  include/fcu_sim_plugins/GPS_plugin.h)
target_link_libraries(GPS_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(GPS_plugin ${catkin_EXPORTED_TARGETS})

add_library(gimbal_plugin
  src/gimbal_plugin.cpp
  include/fcu_sim_plugins/gimbal_plugin.h)
target_link_libraries(gimbal_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(gimbal_plugin ${catkin_EXPORTED_TARGETS})

install(
  TARGETS
    odometry_plugin
    imu_plugin
    altimeter_plugin
    wind_plugin
    gimbal_plugin
    GPS_plugin
    airspeed_plugin
    ROSflight_sil_plugin
    multirotor_forces_and_moments_plugin
    aircraft_forces_and_moments_plugin
    magnetometer_plugin
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
