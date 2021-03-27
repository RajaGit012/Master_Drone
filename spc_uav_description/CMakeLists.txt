cmake_minimum_required(VERSION 2.8.3)
project(spc_uav_description)

find_package(catkin REQUIRED COMPONENTS
  urdf
  xacro
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES spc_uav_description
#  CATKIN_DEPENDS urdf xacro
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY meshes launch rviz urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  
