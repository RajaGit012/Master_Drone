cmake_minimum_required(VERSION 2.8.3)
project(spc_uav_comm)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  mavros_msgs
  actionlib_msgs
  dynamic_reconfigure
  message_generation
  geometry_msgs
)

add_message_files(
  FILES
  ControlSignals.msg
)

add_action_files(
  DIRECTORY action
  FILES LearnParam.action
)
generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs geometry_msgs
)
###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES spc_uav_comm
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
  CATKIN_DEPENDS actionlib_msgs dynamic_reconfigure message_runtime
)

###########
## Build ##
###########
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
#for custom message listening to
#add_dependencies(pointcloud_processing_node ${catkin_EXPORTED_TARGETS})
