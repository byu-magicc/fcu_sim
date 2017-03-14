cmake_minimum_required(VERSION 2.8.3)
project(fcu_sim_plugins)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99")

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
  std_msgs
  tf2_ros
  image_transport
  polled_camera
  camera_info_manager
  nav_msgs
  gazebo_ros
  gazebo_plugins
)

find_package(Eigen3 REQUIRED)
find_package(gazebo REQUIRED)
# find_package(OpenCV REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(lib/ROSflight/include)
include_directories(lib/ROSflight/lib/turbotrig)
include_directories(lib/ROSflight/lib)
include_directories(lib/ROSflight_SIL)

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  CATKIN_DEPENDS cv_bridge geometry_msgs fcu_sim rosbag roscpp
                 fcu_sim std_srvs tf relative_nav
		 cv_bridge geometry_msgs fcu_sim rosbag roscpp gazebo_plugins gazebo_ros
		 tf2_ros image_transport polled_camera camera_info_manager std_msgs nav_msgs sensor_msgs
  DEPENDS eigen gazebo opencv fcu_common SDF roscpp gazebo_ros
)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})

# add_library(ROSflight_sil_plugin
#  src/ROSflight_sil.cpp
#
#  lib/ROSflight/src/rosflight.c
#  lib/ROSflight/src/printf.c
#  lib/ROSflight/src/estimator.c
#  lib/ROSflight/src/mixer.c
#  lib/ROSflight/src/controller.c
#  lib/ROSflight/src/param.c
#  lib/ROSflight/src/mode.c
#  lib/ROSflight/src/mavlink.c
#  lib/ROSflight/src/mavlink_stream.c
#  lib/ROSflight/src/mavlink_receive.c
#  lib/ROSflight/src/mavlink_util.c
#  lib/ROSflight/src/mavlink_param.c
#  lib/ROSflight/src/rc.c
#  lib/ROSflight/src/mux.c
#  lib/ROSflight/src/sensors.c
#  lib/ROSflight/lib/turbotrig/turbotrig.c
#  lib/ROSflight/lib/turbotrig/turbovec.c
#  lib/ROSflight_SIL/board.c)
# target_link_libraries(ROSflight_sil_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} yaml-cpp)
# add_dependencies(ROSflight_sil_plugin ${catkin_EXPORTED_TARGETS} fcu_common_generate_messages_cpp)
# set_property( TARGET ROSflight_sil_plugin APPEND_STRING PROPERTY COMPILE_FLAGS -Wno-format-extra-args )

add_library(aircraft_forces_and_moments_plugin
  src/aircraft_forces_and_moments.cpp
  include/fcu_sim_plugins/aircraft_forces_and_moments.h)
target_link_libraries(aircraft_forces_and_moments_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(aircraft_forces_and_moments_plugin ${catkin_EXPORTED_TARGETS} fcu_common_generate_messages_cpp)

add_library(multirotor_forces_and_moments_plugin
  src/multirotor_forces_and_moments.cpp
  include/fcu_sim_plugins/multirotor_forces_and_moments.h)
target_link_libraries(multirotor_forces_and_moments_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(multirotor_forces_and_moments_plugin ${catkin_EXPORTED_TARGETS} fcu_common_generate_messages_cpp)

add_library(magnetometer_plugin
  src/magnetometer.cpp
include/fcu_sim_plugins/magnetometer.h)
target_link_libraries(magnetometer_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(magnetometer_plugin ${catkin_EXPORTED_TARGETS})

add_library(aircraft_truth_plugin
  src/aircraft_truth.cpp
  include/fcu_sim_plugins/aircraft_truth.h)
target_link_libraries(aircraft_truth_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(aircraft_truth_plugin ${catkin_EXPORTED_TARGETS} fcu_common_generate_messages_cpp)

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

add_library(barometer_plugin
  src/barometer_plugin.cpp
  include/fcu_sim_plugins/barometer_plugin.h)
target_link_libraries(barometer_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(barometer_plugin ${catkin_EXPORTED_TARGETS} fcu_common_generate_messages_cpp)

add_library(wind_plugin
  src/wind_plugin.cpp
  include/fcu_sim_plugins/wind_plugin.h)
target_link_libraries(wind_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(wind_plugin ${catkin_EXPORTED_TARGETS})

add_library(airspeed_plugin
  src/airspeed_plugin.cpp
  include/fcu_sim_plugins/airspeed_plugin.h)
target_link_libraries(airspeed_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(airspeed_plugin ${catkin_EXPORTED_TARGETS} fcu_common_generate_messages_cpp)

add_library(GPS_plugin
  src/GPS_plugin.cpp
  include/fcu_sim_plugins/GPS_plugin.h)
target_link_libraries(GPS_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(GPS_plugin ${catkin_EXPORTED_TARGETS} fcu_common_generate_messages_cpp)

add_library(gimbal_plugin
  src/gimbal_plugin.cpp
  include/fcu_sim_plugins/gimbal_plugin.h)
target_link_libraries(gimbal_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(gimbal_plugin ${catkin_EXPORTED_TARGETS})

add_library(fcu_sim_camera_utils
  src/gazebo_ros_camera_utils.cpp)
target_link_libraries(fcu_sim_camera_utils ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(fcu_sim_camera_utils ${catkin_EXPORTED_TARGETS})

add_library(fcu_sim_camera_plugin
  src/gazebo_ros_camera.cpp)
target_link_libraries(fcu_sim_camera_plugin fcu_sim_camera_utils ${GAZEBO_LIBRARIES} CameraPlugin ${catkin_LIBRARIES})
add_dependencies(fcu_sim_camera_plugin ${catkin_EXPORTED_TARGETS})

add_library(step_camera
  src/step_camera.cpp
  include/fcu_sim_plugins/step_camera.h)
target_link_libraries(step_camera ${catkin_LIBRARIES} ${GAZEBO_libraries} CameraPlugin)
add_dependencies(step_camera ${catkin_EXPORTED_TARGETS})

add_library(world_utilities
  src/world_utilities.cpp
  include/fcu_sim_plugins/world_utilities.h)
target_link_libraries(world_utilities ${catkin_LIBRARIES} ${GAZEBO_libraries})
add_dependencies(world_utilities ${catkin_EXPORTED_TARGETS})

install(
  TARGETS
    odometry_plugin
    imu_plugin
    barometer_plugin
    wind_plugin
    gimbal_plugin
    GPS_plugin
    airspeed_plugin
    # ROSflight_sil_plugin
    multirotor_forces_and_moments_plugin
    aircraft_forces_and_moments_plugin
    magnetometer_plugin
    step_camera
    world_utilities
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
