Installing and Running the GUI alone:
------------------------------------

Prerequisites:
--------------
1. Installation of Ubuntu 16.04

2. Installation of ROS Kinetic desktop full

3. Setup a new ROS workspace

4. Setup catkin-tools and wstool

5. Sign-up an account for gitlab


Installation Settings:
----------------------

Install Mavros
    $ sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras
    $ wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
    $ sudo ./install_geographiclib_datasets.sh

    - Download the packages into the src folder:
    $ cd ~/catkin_ws/src/
    $ git clone https://gitlab.com/ut_spectors/spc_uav_systems.git
    
    - Build only Related Package --> spc_uav_comm:
    $ catkin build spc_uav_comm
    
There should be no building errors.

For displaying the GUI's properly we need to install also some standard software:

    - Open a terminal and type:
    $ sudo apt-get install python-gi-cairo xdot python-wxtools

In order to be able to adapt the interfaces, you need a program called glade. Let's install this one as well (regardless if you are going to need it):

    - In a terminal type:
    $ sudo apt-get install glade
    
    For more information on this program: https://glade.gnome.org/



Running GUI:
------------
    $ roslaunch spc_uav_comm gui_omni_uav.launch
    
To view the frames you can use RVIZ --> (http://wiki.ros.org/rviz)

    $ rosrun rviz rviz 
    
    
    
    
IN CASE This guideline is missing something, contact r.a.m.rashadhashem@utwente.nl

