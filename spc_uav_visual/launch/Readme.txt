helix.launch
Launches helix.cpp file in src folder
Performs helical trajectory if GUI "External Override" is pressed
Performs octree segmentation and determines Next Best Views(NBV)


Inspection.launch
Launches Inspection.cpp file in src folder
Change the orientation of UAV
Perform trajectory to all the NBVs when GUI "Inspection override is pressed"


Icp.launch
Input one - Source point cloud folder path
Input two - target point cloud folder path
ICP fitness score is obtained


cams_plot_re_broadcaster.launch
Launches cams_plot_re_broadcaster.cpp file in src folder
Generate the position and orientation of UAV
Generate the position and orientation of estimated camera trajectory
Use these information to polt the UAV path and estimated camera trajectroy using rosbag