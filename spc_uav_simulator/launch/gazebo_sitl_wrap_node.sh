#!/bin/bash
# My first script

echo ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Starting Gazebo World!<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"
roscd px4
no_sim=1 make posix_sitl_default gazebo
source ~/.bashrc
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/posix_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch spc_uav_simulator posix_sitl_gazebo_direct.launch vehicle:=alpha


cd ~/spc_catkin_ws/src/spc_uav_systems/Firmware
no_sim=1 make posix_sitl_default gazebo
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/posix_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch spc_uav_simulator posix_sitl_gazebo.launch vehicle:=alpha


# Use after any modification done to code !
make clean 



# Build for pixhawk
make px4fmu-v3_default

