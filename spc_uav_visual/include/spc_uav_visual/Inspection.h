#include "spc_uav_visual/pointcloud_processing_node.h"

#include <boost/foreach.hpp>
#include <iostream>
#include <vector>

#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <spc_uav_visual/my_msg.h>

#include <bits/stdc++.h>
#include <iomanip>
#include <iostream>
#include <math.h>

#include <spc_uav_visual/cloud_segment.h>

#include <cmath>

//for logfile creation
#include <iomanip>
#include <sstream>
#include <chrono>
#include <ctime>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <boost/foreach.hpp>
#include <iostream>
#include <vector>

#include "geometry_msgs/Point.h"
#include <spc_uav_visual/my_msg.h>

#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <cmath>

//for logfile creation
#include <iomanip>
#include <sstream>
#include <chrono>
#include <ctime>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <std_msgs/Int32.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree_container.h>
#include <pcl/octree/octree_pointcloud_pointvector.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int64.h>

#include <iostream>
#include <vector>
#include <ctime>
#include <string>
#include <iostream>
#include <vector>
#include <ctime>
#include <pcl/filters/passthrough.h>
#include "std_msgs/String.h"

void updatePosMsg();
void reInitialize();
void transition();
void setPrePose();
void setCircle(float start_angle, float dest_angle, bool direction);
void tocircle(float minangle);
//void FLYTOPOINTS(float x, float y, float z);
void flytopoints(float start_angle, float dest_angle);
void Inspection();
void flycircular();
double trajDur();
ros::Publisher set_p_insp_pub_;
ros::Publisher rawpoint_pub;
spc_uav_visual::InspPos Insp_pos_msg_;
//ros::Publisher chatter_pub;
float tx, ty, tz, Ix, Iy, Iz, dist, tpitch, Wx, Wy, Wz;
float cur_x_, cur_y_, cur_z_, cur_roll_, cur_pitch_, cur_yaw_, cur_phi_, cur_dir_;
float pre_x_, pre_y_, pre_z_, pre_roll_, pre_pitch_, pre_yaw_;
float pre_r_, dest_angle, start_angle;
bool gui_flag_, direction;
bool Insp_;
bool transition_ = true;
int counter = 0;
float w = 0.0;
float angle_tocover, angle, minangle;
float setpoint_oncircle;
float pre_phi_ = M_PI;
std::vector<control_api_waypoint> waypointList;
ros::Time t_start_traj_;
float t_traj = 30.0;
float t_waypoint = 30.0;
float t_height = 15.0;
float t_wait = 10.0;
float t_end = 1000.0;
std_msgs::Int64 msg;
static int old_NoP = 0;

unsigned int count = 1;

