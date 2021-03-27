cmake_minimum_required(VERSION 2.8.3)
project(spc_uav_simulator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
    cmake_modules
  mavros_msgs
  tf
  tf2_ros
  geometry_msgs
  tf_conversions
  mav_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS
    LIBRARIES spc_uav_simulator
    CATKIN_DEPENDS  roscpp tf tf2_ros mavros_msgs mav_msgs tf_conversions
    DEPENDS system_lib
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)


add_executable(simulation_publisher src/simulation_publisher.cpp)
target_link_libraries(simulation_publisher ${catkin_LIBRARIES})
