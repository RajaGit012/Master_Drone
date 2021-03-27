//
// Created by patrick on 12-3-19.
//

#ifndef INCLUDE_SPC_UAV_VISUAL_POINTCLOUD_PROCESSING_NODE_H
#define INCLUDE_SPC_UAV_VISUAL_POINTCLOUD_PROCESSING_NODE_H

#include <spc_uav_visual/pcp_common.h>
#include <spc_uav_visual/pointcloud_processor.h>
#include <ros/ros.h>

#include <sensor_msgs/point_cloud_conversion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"

#include <std_msgs/Bool.h>
#include <spc_uav_visual/PcPos.h> //custom defined message
#include <spc_uav_visual/InspPos.h>
#include <spc_uav_visual/Numpoints.h>
#include <spc_uav_visual/my_msg.h>

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


#include <iostream>
#include <vector>
#include <ctime>

#include <iostream>
#include <vector>
#include <ctime>
#include <pcl/filters/passthrough.h>
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
#include <string>

#include <fstream>
std::string Helix_complete;
struct control_api_waypoint{
     float x; 
     float y; 
     float z;  
 };

typedef pcl::PointCloud<pcl::PointXYZ> CloudInsp;
namespace pcp {  

class PointCloudProcessingNode {
 public:
  PointCloudProcessingNode(ros::NodeHandle &node_handle);
  ~PointCloudProcessingNode(){
    delete tf_list_;
  }

  ros::NodeHandle node_handle_;

  std::string sub_p_cloud_top_; //subscribed point cloud topic
  std::string cloud_lowDens; //subscribed point cloud topic

  void OrbslamFilterCallback(const PointCloud::ConstPtr &input_cloud);

  void pcInspCallback(const PointCloud::ConstPtr &cloud);

  //void OrbslamFilterCallback(const PointCloud::ConstPtr &cloud_filter);

  void InspecPointsCallback(const  sensor_msgs::PointCloud2ConstPtr &inspec_cloud);

  //void InspecPointsCallback(const spc_uav_visual::my_msg::ConstPtr& msg);

  void waitCallback(const nav_msgs::Odometry::ConstPtr& msg);

  void InitParams();
  //recieves if override mode in gui is active
  void ovModeGuiCallback(const std_msgs::Bool::ConstPtr &msg);

  void publishTrajectoryTimerCallback(const ros::TimerEvent&);

  void InspectionTimerCallback(const ros::TimerEvent&);

  void lookupCurSetPoint();
  



 private:
  PointCloudProcessingNode();

  std::unique_ptr<PointCloudProcessor> pc_proc_;

  enum TraState {INITPOS, //fly to initial position
      RST_SLAM, //reset ORB-SLAM2
      HELIX, //fly an helix like in typical/classic approaches
      NEGHELIX,
      INIT_CIRC, //fly initial circle to estimate object and close loop with orbslam 2
      LOW_ENT, //fly to segment with lowest entropy
      NEXT_SEGMENT, //fly to next segment (first after lowest entropy is the neighbour segment with the lowest entropy and this also defines the further direction the segments are visited)
      INSP_SEGMENT, //fly inspection path at segment
      SEC_INSPECTION,// fly to low density area and do inspection
      INSPEC,
      WAIT, //wait for drone/controller to get to setpoint
      END}; //do nothing
  TraState cur_fl_state_;
  TraState prev_fl_state_;
  
  bool transition_; //to help executing code only after transition/first entering state

  std::string name_of_node_;

  double obj_center_x_, obj_center_y_; //object of interest center
  double obj_dim_x_, obj_dim_y_; //object dimensions for cropping and segment creation

  int k_neighbours_; //knn neighbour search's number of neighbours taken into account

  bool gui_flag_; //external mode active in gui
  bool fly_; //if trajectory should be followed
  bool fly_helix_; //true: helix is flight otherwise autonomous inspection
  bool helix_state_done, go_to_sleep; //true: end of helix fligh notified to start inspection timer
  bool fly_seg_insp_; //true: fly segment inspection path
  bool top_; //true: when a upper z limit when flying autonomous inspection

  ros::Publisher pc_pub_; //pointcloud publisher
  ros::Publisher set_p_pub_; //setpoint publisher
  
  std::string pub_p_cloud_top_; //published point cloud topic
  std::string pc_frame_; //parent frame of published point cloud
  std::string set_point_frame; //parent frame id of setpoint
  std::string set_point_child_frame; //child frame id of setpoint
  std_msgs::String str;

  //TODO: obsolete?
  geometry_msgs::TransformStamped cur_setp_; //setpoint listened from TF
  geometry_msgs::TransformStamped prev_setp_; //previous setpoint

  spc_uav_visual::PcPos pos_msg_;
  spc_uav_visual::InspPos Insp_pos_msg_;
  spc_uav_visual::Numpoints data;

  // TF Buffer
  tf2_ros::Buffer tf_buffer_;
  // TF listener
  tf2_ros::TransformListener* tf_list_;

  ros::Time t_start_;
  ros::Time t_start_traj_;

  double w_des_;//desired angular velocity for rotational flights
  int n_rot_des_; //number of desired rotations for helix flight
  double h_diff_; //height difference from start z to object of interest height
  float v_z_helix_; //required velocity in z direction to reach object of interest height within the number of rotations
  float T_; //period of rotation
  double r_; //radius
  double v_z_insp_; //absolute velocity in z direction for inspection path
  


  float cur_x_, cur_y_, cur_z_, cur_roll_, cur_pitch_, cur_yaw_;
  float cur_r_, cur_phi_;
  float pre_x_, pre_y_, pre_z_, pre_roll_, pre_pitch_, pre_yaw_;
  float pre_r_, pre_phi_;
  float tx,ty,tz,Ix,Iy,Iz,dist;
  float rp_ ; //radius in polar coordinate

  float minX = 1.7, minY = -1.3, minZ = -0.1;
  float maxX = 4.3, maxY = +1.1, maxZ = +8.1;

  float w_; //angular velocity in rad/s for flying the trajectory's angular distance in the given time (t_traj) (if flying further than 1 rotation it is scaled)
  bool dir_; //flight direction - true = counter clockwise / positive

  int cur_seg_id_; //segment currently inspected
  int seg_counter_; //starts from number of segments and counts down until all are visited

  double t_init_; //time in sec to reach initial position
  double t_rst_slam_; //time to wait until ORB-SLAM2 should be reseted
  double t_wait_; //transition time between trajectories to let controller reach setpoint
  double t_helix_; //helix execution time
  double t_init_circ_; //time in sec for flying the initial circle of optimized reconstruction algorithm
  double t_ent_seg_; //dynamically set time based on distance from start to segment with lowest entropy
  double t_inter_seg_; //time to fly from one segment to another resp. an angle of pi/2 / 90 degrees
  double t_full_alg_flight_; //duration of the full alorithm flight
  double t_seg_insp_; //duration for inspecting a segment
  long double theta,det,dot,beta,uav_cur_angle;
  
  double init_z_;
  double helix_z_;
  double init_x_;
  float init_phi_,dfc; //angle at which initial circle or the helix gets started

  bool init_circ_dir_; //true pos/ccw direction of initial circle

  std::ofstream log_file_; //log csv file for documenting
  int step_counter_; //counts algorithmic steps for logfile


  //determines current sate used by calcSetPoint
  void flightStateMachine();

  void calcHelixSetPoint();

  void NegcalcHelixSetPoint();

  void INITIALPOSE();

  void rstSLAM();

  void calcIntCircSetpoint();

  //fly to segment center with lowest entropy
  void calcToLowEntropySetpoint();

  //fly to next segment which has to be investigated
  void calcToNextSegSetpoint();

  //fly inspection path at segment (currently going down h_diff if on top and if at low down going up)
  void calcSegInspSetpoint();

  void updatePosMsg();

  void setPrePose();

  //returns time passed since trajectory start
  double trajDur();

  //transition betwenn flight states
  void transition();

  void reInitialize();

  // fly from start/current angle to destination angle taking number of rotations and direction into account
  // therefore calculating for the given trajectory time the appropriate angular velocity to reach the point in the given time
  // dir_ has to be set before (true = counter clockwise / positive)
  void setCircularParams(float phi_st, float phi_dst, float t_traj, unsigned int n_rot = 0);

  // fly from current angle n rotations
  // therefore calculating for the given trajectory time the appropriate angular velocity to finish the rotations in the given time
  // dir_ has to be set before (true = counter clockwise / positive)
  void setCircularParams(float t_traj, unsigned int n_rot);

  // always call before (=after state transition) a setCircularParams!
  // flys on (unit) cicrle around object 
  // dir_ has to be set before (true = counter clockwise / positive)
  void flyCircular();

  void toCarthesian();

  void FLYTOPOINTS(float x, float y, float z);

  void STAY();

  //creates logfile in /home/USERNAME/.ros
  void createLogFile();

  //writes step result to the file created in createLogFile
  void writeStepResult();

  void DelayPointsCallback();

};

}

#endif //INCLUDE_SPC_UAV_VISUAL_POINTCLOUD_PROCESSING_NODE_H
