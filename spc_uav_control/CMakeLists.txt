cmake_minimum_required(VERSION 2.8.3)
project(spc_uav_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cmake_modules
  message_generation
  mavros_msgs
  tf
  tf2_ros
  geometry_msgs
  gazebo_ros_control
  genmsg
  dynamic_reconfigure
  tf_conversions
  mav_msgs
  actionlib_msgs
  spc_uav_comm
)

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################
## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  config/cfg/SO3_Control.cfg
  config/cfg/SE3_Control.cfg
  config/cfg/Admittance_Control.cfg
  config/cfg/Impedance_Control.cfg
  config/cfg/ET_Impedance_Control.cfg
)

###########
## Build ##
###########
catkin_package(
    INCLUDE_DIRS include ${EIGEN_INCLUDE_DIRS}
    LIBRARIES spc_uav_control
    CATKIN_DEPENDS message_runtime cmake_modules roscpp rospy tf dynamic_reconfigure tf2_ros mavros_msgs genmsg mav_msgs tf_conversions actionlib_msgs spc_uav_comm
    DEPENDS system_lib Eigen 
)

include_directories(
    include ${catkin_INCLUDE_DIRS}
)


#################################
### SE3 Controller Declaration ##
#################################
## Declare a C++ library
 add_library(geometric_SE3_controller src/geometric_SE3_controller.cpp src/MathHelper.cpp src/controller_node.cpp)
 
# Declare a C++ executable
add_executable(geometric_SE3_controller_node src/geometric_SE3_controller_node.cpp)
add_dependencies(geometric_SE3_controller_node geometric_SE3_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(geometric_SE3_controller_node
  ${catkin_LIBRARIES}
  geometric_SE3_controller
)

########################################
### Admittance Controller Declaration ##
########################################
## Declare a C++ library
 add_library(geometric_adm_controller src/geometric_SE3_controller.cpp src/geometric_adm_controller.cpp src/MathHelper.cpp src/controller_node.cpp src/geometric_SE3_controller_node.cpp)
 
# Declare a C++ executable
add_executable(geometric_admittance_controller_node src/geometric_adm_controller_node.cpp)
add_dependencies(geometric_admittance_controller_node geometric_adm_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(geometric_admittance_controller_node
  ${catkin_LIBRARIES}
  geometric_adm_controller
)

#######################################
### Impedance Controller Declaration ##
#######################################
## Declare a C++ library
 add_library(geometric_impedance_controller src/geom_impedance_controller.cpp src/MathHelper.cpp src/controller_node.cpp)
 
# Declare a C++ executable
add_executable(geometric_impedance_controller_node src/geom_impedance_controller_node.cpp)
add_dependencies(geometric_impedance_controller_node geometric_impedance_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(geometric_impedance_controller_node
  ${catkin_LIBRARIES}
  geometric_impedance_controller
)

###############################################
### Underactuated SO3 Controller Declaration ##
###############################################

## Declare a C++ library
 add_library(geometric_SO3_controller src/geometric_SO3_controller.cpp src/MathHelper.cpp src/controller_node.cpp)
 
# Declare a C++ executable
add_executable(geometric_SO3_controller_node src/geometric_SO3_controller_node.cpp)
add_dependencies(geometric_SO3_controller_node geometric_SO3_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(geometric_SO3_controller_node
  ${catkin_LIBRARIES}
  geometric_SO3_controller
)

#########################################################
### Energy Tank Based Impedance Controller Declaration ##
#########################################################
## Declare a C++ library
 add_library(etank_impedance_controller src/etank_impedance_controller.cpp src/MathHelper.cpp src/controller_node.cpp src/energy_tank.cpp)
 
# Declare a C++ executable
add_executable(etank_impedance_controller_node src/etank_impedance_controller_node.cpp)
add_dependencies(etank_impedance_controller_node etank_impedance_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(etank_impedance_controller_node
  ${catkin_LIBRARIES}
  etank_impedance_controller
)


