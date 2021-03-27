SPECTORS UAV SYSTEMS
====================

This is the private repository for UTwente-RAM members of the SPECTORS project. The project focuses on the design of Unmanned Aerial Vehicles (UAVs) for interacting with the environment.

The RotorS gazebo simulator and the px4-MAVLINK gazebo simulator packages are used within the project.

Below we provide installation instructions to get started !


Pre-requisites for the following are: (refer to https://wiki.ram.ewi.utwente.nl/index.php/Installation_and_setup_of_ROS)

    1. Installation of Ubuntu 16.04

    2. Installation of ROS Kinetic desktop full

    3. Setup a new ROS workspace

    4. Setup catkin-tools and wstool

    5. Sign-up an account for gitlab, and setup an SSH

PX4 simulator:
--------------

1) Follow the instructions on https://wiki.ram.ewi.utwente.nl/index.php/Setting_up_PX4_SITL

2) If you place the Firmware folder inside your catkin workspace, then you don't want to build the PX4 firmware using catkin, so add the px4 package to the blacklist by typing
```bash
$ catkin config --blacklist px4
```
3) Install QGroundControl:

    - Go to http://qgroundcontrol.com/ to find more information on QGroundControl.

    - To download the software find the download link on that page

    - Download the AppImage for Linux.

    - Open a terminal and move the appimage from Downloads to your home-folder then give the app-image execute rights:
```bash    
    $ mv ~/Downloads/QGroundControl.AppImage ~/   
    $ chmod +x ~/QGroundControl.AppImage
```
4) Run QGC to test it:

    - Open a terminal

    - Start the program:
```bash
    $ ./QGroundControl.AppImage
```
5) At this point the px4 simulator should work when you run the "posix_sitl.launch"


RotorS simulator:
-----------------
These are part of the instructions on https://github.com/ethz-asl/rotors_simulator which include pre-requisites 2-4.

1) Install and initialize additional ROS packages, catkin-tools, and wstool:
```bash
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
$ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install ros-kinetic-joy ros-kinetic-octomap-ros ros-kinetic-mavlink ros-kinetic-control-toolbox libgoogle-glog-dev
$ sudo rosdep init
$ sudo rosdep fix-permissions
$ rosdep update
$ source /opt/ros/kinetic/setup.bash
```
2) Clone the RotorS Repositories and Install

```bash
$ cd ~/catkin_ws/src
$ wget https://raw.githubusercontent.com/ethz-asl/rotors_simulator/master/rotors_minimal.rosinstall
$ wstool merge rotors_minimal.rosinstall
$ wstool update
```


3) Build your workspace

```bash
$ cd ~/catkin_ws/
$ catkin build
```

4) Add sourcing to your .bashrc file

```bash
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

5) Some problems might be caused by the rotors_hil_interface, so a proposed solution is to create an empty file in the rotors_hil_interface folder named "CATKIN_IGNORE"

6) Launch the simulator to test the success of the installation
```bash
$ roslaunch rotors_gazebo mav_hovering_example.launch mav_name:=firefly world_name:=basic
```

SPC Packages:
-------------

1) Request Gitlab Access from Ramy Rashad (r.a.m.rashadhashem@utwente.nl)

2) Clone the SPC packages in you src directory

```bash
$ cd ~/catkin_ws/src
$ git clone https://gitlab.com/RajaGitter/master-thesis.git
```

3) Clone the Multi-Sensor-Fusion State-estimator package from ETH
```bash
$ git clone https://github.com/ethz-asl/ethzasl_msf.git
```

4) Download also these packages

```bash
$ git clone https://github.com/ethz-asl/glog_catkin.git
$ git clone https://github.com/catkin/catkin_simple.git
$ git clone https://github.com/ethz-asl/gflags_catkin.git
$ git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git
$ git clone https://github.com/ros/geometry.git
$ git clone https://github.com/ros/geometry2.git
```

5) Install the motion-capture software ROS-package (for Experiments)
```bash
    $ git clone https://github.com/ros-drivers/mocap_optitrack.git
```

6) Install the following software in a new terminal
```bash
$ sudo apt-get install python-gi-cairo xdot python-wxtools
$ sudo apt-get install glade
$ sudo apt-get install autoconf
```

7) To use the Impedance Learning Module, you should also install

```bash
$ sudo pip install --upgrade pip
$ sudo pip install -U scikit-learn
$ sudo pip install pyDOE
```

8) To use the Visual Module, you should also

```bash
$ cd ~/catkin_ws/src
$ git clone https://gitlab.com/ut_spectors/orb_slam_2_ros.git
$ git checkout spc_uav_o_slam
$ catkin build orb_slam2_ros
```
9) It is recommended to gain proper performance/reliable tracking/mapping to also install everything necessary for GPU acceleration
    - (e.g. CUDA) of the used OpenCV library
    - (e.g. graphic card driver of the manufacturer)

Running a Demo:
---------------
To test the success of the installation, run ONE of launch files in the "RotorS" folder for the RotorS simulator or one of launch files in the "PX4" folder for the Px4 simulator: e.g
```bash
$ roslaunch spc_uav_main R1_SO3_Underactuated_Control_Alpha.launch
$ roslaunch spc_uav_main P1_SO3_Underactuated_Control_Alpha.launch
$ roslaunch spc_uav_main R8_ETank_Impedance_Control_with_rviz.launch    
```

To use the Visual Module:
    - In a new terminal
```bash
    $ roslaunch orb_slam2_ros  orb_slam2_stereo.launch
```
    - In another terminal run
```bash
    $ roslaunch spc_uav_visual helix.launch
```
    - In another terminal run
```bash
    $ roslaunch spc_uav_visual Inspection.launch
```
    - In the GUI one has to click on the radio button "external override" and the UAV will start flying according to the parameters in the helix.launch file

    - In the GUI one has to click on the radio button "Inspection override" when the helical trajectory is complete. The UAV will start flying to the inspection points generated from Inspection.launch

If you use a PX4 demo, then you must run the commands in the gazebo_sitl_wrap_node.sh file in the spc_uav_simulator package

To fly, you need to "ARM" the UAV and switch to "OFFBOARD" mode from the GUI.




Description of the SPC Packages:
--------------------------------
1) spc_uav_comm: contains the definitions of messages, services and actions. In addition to the GUI related files.

2) spc_uav_control: contains all the flight control algorithms and corresponding ROS wrapping nodes

3) spc_uav_description: contains the meshes and URDF files of the SPECTORS UAVs. The Current fleet contains Alpha, Beta, BetaX, and Gamma

4) spc_uav_main: contains the main launch files used for simulations, experiments, and replaying experiments.

5) spc_uav_simulator: contains the gazebo launch files, the px4-sitl launch files and the gazebo simulation files.

6) spc_uav_visual: contains launch files for the helical trajectory, octree segmentation of ORB-SLAM2, determining Next Best View points and launch files that enables the exploratory trajectory to Next Best View points flying different reconstruction trajectories

Setting up Eclipse:
-------------------
 1) Installation and setup:

 Follow exactly the steps under: Install Eclipse in the pdf:

 https://www.ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/ROS2017/how_to_setup_developer_pc.pdf

 2) Creating the Eclipse project files:

 Don't forget to add the additional build flags to your catkin config to generate the eclipse related files.
 These build flags generates the build files that you import next in Eclipse:


 3) Follow steps 3 and 4 in http://wiki.ros.org/IDEs, under the Eclipse setup instructions
 These instructions should allow you to import the build files of the project you're interested in generated from the catkin_build.
 Do not import all of your workspace, only the projects that you will be modifying or navigating.
 It is not necessary to build projects from inside Eclipse, you can do that easier from the terminal using catkin build.


Updates:
--------
If you have any updates to the previous instructions or missing ones, please send an email to (r.a.m.rashadhashem@utwente.nl) to update the instructions.


Cool Courses:
-------------
If you want to boost your skills in the following subjects, these are some online courses that you can start from:

1) Terminal Commands
https://eu.udacity.com/course/shell-workshop--ud206

2) Version Control with Git
https://eu.udacity.com/course/version-control-with-git--ud123

3) Github and Collaboration
https://eu.udacity.com/course/github-collaboration--ud456

4) Writing READMEs (Markdown)
https://eu.udacity.com/course/writing-readmes--ud777
