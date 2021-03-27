
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
#include <std_msgs/Int64.h>

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
#include "std_msgs/String.h"

using namespace pcp;
using namespace std;
std_msgs::Int64 msg;

typedef pcl::PointCloud<pcl::PointXYZ> CloudInsp;

ros::Publisher pub_filterCloud;
ros::Publisher pub;
ros::Publisher pub_onSurface;
ros::Publisher pub_Helixcomplete;
ros::Publisher NoP_pub;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "helix");
  ros::NodeHandle nh;

  PointCloudProcessingNode pc_node(nh);
  // subscribe to the GUI input esternal overide
  ros::Subscriber sub_ov = nh.subscribe("gui/extOvRide", 1, &PointCloudProcessingNode::ovModeGuiCallback, &pc_node);
  // subscribe to ORBSLAM2 output
  ros::Subscriber sub_pc = nh.subscribe<PointCloud>(pc_node.sub_p_cloud_top_, 1, &PointCloudProcessingNode::OrbslamFilterCallback, &pc_node);
  // subscribe to current position and orientation of UAV
  ros::Subscriber Delay_ov = nh.subscribe("/ground_truth/odometry", 1, &PointCloudProcessingNode::waitCallback, &pc_node);

  //publish setpoint to gui at 100Hz
  ros::Timer timer = nh.createTimer(ros::Duration(0.01), &PointCloudProcessingNode::publishTrajectoryTimerCallback, &pc_node);
  // Publish the points to be inspected
  pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/points_to_inspect", 1);
  // publish the corresponding points to be inspected on the object surface
  pub_onSurface = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/points_on_surface", 1);
  // publish the crop box filtered point cloud
  pub_filterCloud = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/cropped_cloud", 1);
  // publish the status of flight,  whether the helical trajectpry is complete or not
  pub_Helixcomplete = nh.advertise<std_msgs::String>("/Helix_Status", 1);
  // publish total number of points in point cloud
  NoP_pub = nh.advertise<std_msgs::Int64>("/Number_of_Points", 1000);

  ros::spin();
}

PointCloudProcessingNode::PointCloudProcessingNode(ros::NodeHandle &node_handle)
{
  node_handle_ = node_handle;
  name_of_node_ = ros::this_node::getName();
  tf_list_ = new tf2_ros::TransformListener(tf_buffer_);

  cur_x_ = -3.0;
  cur_y_ = cur_z_ = cur_roll_ = cur_yaw_ = 0.0;
  cur_pitch_ = 0.0; //-(tz*2.0);
  pre_x_ = pre_y_ = pre_z_ = pre_roll_ = pre_yaw_ = 0.0;
  pre_pitch_ = 0.0; //-(tz*2.0);
  cur_r_ = 3.0;
  cur_phi_ = M_PI;
  pre_r_ = 0.0;
  pre_phi_ = M_PI;
  w_ = 0.0;
  dir_ = true;

  cur_seg_id_ = -1;
  seg_counter_ = 4;

  InitParams();

  pc_pub_ = node_handle_.advertise<PointCloud>(name_of_node_ + pub_p_cloud_top_, 500);
  set_p_pub_ = node_handle_.advertise<spc_uav_visual::PcPos>(name_of_node_ + "/set_p", 1);

  v_z_helix_ = 0.1;
  T_ = 0.0;

  t_rst_slam_ = 1.0;

  init_circ_dir_ = true;

  t_helix_ = 10.0; //currently getting overwritten! in dependence of w_des_

  t_init_circ_ = 10.0; //currently getting overwritten! in dependence of w_des_

  t_inter_seg_ = 3.5; //currently getting overwritten! in dependence of w_des_

  t_seg_insp_ = 0.0; //gets set at state transition in dependence of v_z_insp_

  t_full_alg_flight_ = 0.0;

  t_wait_ = 3.0;

  gui_flag_ = false;
  fly_ = false;
  go_to_sleep = false;
  helix_state_done = true;
  top_ = false; //gets set true at transition before first call

  cur_fl_state_ = INITPOS;
  prev_fl_state_ = INITPOS;
  transition_ = false;

  t_start_ = ros::Time(0);      //gets set when gui flag received
  t_start_traj_ = ros::Time(0); //gets set when gui flag received

  step_counter_ = -1;
}

void PointCloudProcessingNode::InitParams()
{

  node_handle_.param<std::string>(name_of_node_ + "/sub_p_cloud_top", sub_p_cloud_top_, "/orb_slam2_stereo/map_points");
  node_handle_.param<std::string>(name_of_node_ + "/pub_p_cloud_top", pub_p_cloud_top_, "/seg_cloud");
  node_handle_.param<std::string>(name_of_node_ + "/pc_frame", pc_frame_, "world");
  node_handle_.param<std::string>(name_of_node_ + "/set_point_frame", set_point_frame, "world");
  node_handle_.param<std::string>(name_of_node_ + "/set_point_child_frame", set_point_child_frame, "setpoint");
  node_handle_.param(name_of_node_ + "/obj_center_x", obj_center_x_, 3.0);
  node_handle_.param(name_of_node_ + "/obj_center_y", obj_center_y_, 0.0);
  node_handle_.param(name_of_node_ + "/obj_dim_x", obj_dim_x_, 2.1); //+0.1 due inaccuracy of orbslam2
  node_handle_.param(name_of_node_ + "/obj_dim_y", obj_dim_y_, 2.1);
  node_handle_.param(name_of_node_ + "/fly_helix", fly_helix_, true);
  node_handle_.param(name_of_node_ + "/w_des", w_des_, 0.15);
  node_handle_.param(name_of_node_ + "/n_rot_des", n_rot_des_, 200);
  node_handle_.param(name_of_node_ + "/radius", r_, 3.0);
  node_handle_.param(name_of_node_ + "/t_init", t_init_, 40.0);
  node_handle_.param(name_of_node_ + "/t_helix", t_helix_, 1.0);
  node_handle_.param(name_of_node_ + "/init_x", init_x_, -2.0);
  node_handle_.param(name_of_node_ + "/init_z", init_z_, 3.0);
  node_handle_.param(name_of_node_ + "/helix_z", helix_z_, 4.0);
  node_handle_.param(name_of_node_ + "/h_diff", h_diff_, 3.0 - 1.5);
  node_handle_.param(name_of_node_ + "/v_z_insp", v_z_insp_, 0.15);
  node_handle_.param(name_of_node_ + "/fly_seg_insp", fly_seg_insp_, true);
}

//check if the GUI fla is set
void PointCloudProcessingNode::ovModeGuiCallback(const std_msgs::Bool::ConstPtr &msg)
{
  static bool prev_flag = false;

  prev_flag = gui_flag_;
  gui_flag_ = msg->data;

  if (!prev_flag && gui_flag_)
  {
    reInitialize();
    fly_ = true;
    ROS_INFO("PCProc: GUI changed mode to override");
  }
  else if (prev_flag && !gui_flag_)
  {
    reInitialize();
    fly_ = false;
    ROS_INFO("PCProc: GUI stopped override");
    //TODO: reset behaviour would have to be done in gui
  }
}

void PointCloudProcessingNode::publishTrajectoryTimerCallback(const ros::TimerEvent &)
{
  if (fly_)

    flightStateMachine();

  updatePosMsg();
  set_p_pub_.publish(pos_msg_);
}

void PointCloudProcessingNode::flightStateMachine()
{
  if (cur_fl_state_ == INITPOS)
  {
    if (ros::Time::now().toSec() < (t_start_traj_.toSec() + t_init_))
    {
      INITIALPOSE();
      //helix_state_done = false;
    }
    else
    {
      //helix_state_done = true;
      //go_to_sleep = true;
      cur_fl_state_ = WAIT;
      prev_fl_state_ = INITPOS;
      transition();
    }
  }

  else if (cur_fl_state_ == HELIX)
  {
    if (ros::Time::now().toSec() < (t_start_traj_.toSec() + t_helix_))
    {
      calcHelixSetPoint();
      helix_state_done = false;
      //std::cout << helix_state_done << std::endl;
    }
    else
    {
      helix_state_done = true;
      go_to_sleep = true;
      cur_fl_state_ = WAIT;
      prev_fl_state_ = HELIX;
      transition();
    }
  }

  // else if(cur_fl_state_ == NEGHELIX) {
  //   if (ros::Time::now().toSec() < (t_start_traj_.toSec() + t_helix_ )){
  //     NegcalcHelixSetPoint();
  //     helix_state_done = false;
  //     //std::cout << helix_state_done << std::endl;
  //   }
  //   else {
  //     helix_state_done = true;
  //     go_to_sleep = true;
  //     cur_fl_state_ = WAIT;
  //     prev_fl_state_ = NEGHELIX;
  //     transition();
  //   }
  // }

  else if (cur_fl_state_ == INSPEC)
  {
    //helix_state_done = true;
    //std::cout << helix_state_done << std::endl;
  }

  else if (cur_fl_state_ == WAIT)
  {
    if (ros::Time::now().toSec() < (t_start_traj_.toSec() + t_wait_))
    {
    }
    else
    {
      if (prev_fl_state_ == INITPOS)
      {
        cur_fl_state_ = HELIX;
        prev_fl_state_ = WAIT;
      }
      else if (prev_fl_state_ == HELIX)
      {
        cur_fl_state_ = INSPEC; //INSPEC;
        prev_fl_state_ = WAIT;
      }

      // else if(prev_fl_state_ == NEGHELIX)
      // {
      //   cur_fl_state_ = INSPEC;//INSPEC;
      //   prev_fl_state_ = WAIT;
      // }
      transition();
    }
  }
}

void PointCloudProcessingNode::transition()
{
  setPrePose();
  t_start_traj_ = ros::Time::now();
  transition_ = true;
}

void PointCloudProcessingNode::INITIALPOSE()
{
  cur_x_ = -3.0;
  cur_y_ = 0.0;
  cur_z_ = init_z_ / t_init_ * trajDur();
}

void PointCloudProcessingNode::calcHelixSetPoint()
{
  flyCircular();
  cur_z_ = helix_z_ / t_helix_ * trajDur() + pre_z_;
}

void PointCloudProcessingNode::NegcalcHelixSetPoint()
{
  flyCircular();
  cur_z_ = -helix_z_ / t_helix_ * trajDur() + pre_z_;
}

void PointCloudProcessingNode::waitCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  tx = msg->pose.pose.position.x;
  ty = msg->pose.pose.position.y;
  tz = msg->pose.pose.position.z;
}

double PointCloudProcessingNode::trajDur()
{
  return (ros::Time::now() - t_start_traj_).toSec();
}

void PointCloudProcessingNode::setCircularParams(float phi_st, float phi_dst, float t_traj, unsigned int n_rot)
{
  float phi_dir; //difference angle in rotation direction between start and destination +2PI for each rotation

  if (dir_ && (phi_st < phi_dst))
  {
    phi_dir = phi_dst - phi_st;
  }
  else if (dir_ && (phi_st > phi_dst))
  {
    phi_dir = (2 * M_PI) + phi_dst - phi_st;
  }
  else if ((!dir_) && (phi_st > phi_dst))
  {
    phi_dir = phi_st - phi_dst;
  }
  else if ((!dir_) && (phi_st < phi_dst))
  {
    phi_dir = (2 * M_PI) + phi_st - phi_dst;
  }
  else
  {
    phi_dir = 0.0;
  }

  //  std::cout << "st  : rad: " << phi_st << " = " << phi_st/M_PI << "pi deg: " <<  phi_st/M_PI*180 << std::endl;
  //  std::cout << "dst : rad: " << phi_dst << " = " << phi_dst/M_PI << "pi deg: " <<  phi_dst /M_PI*180 << std::endl;
  //  std::cout << "dir : rad: " << phi_dir << " = " << phi_dir/M_PI << "pi deg: " <<  phi_dir/M_PI*180 << std::endl;

  phi_dir += (2 * M_PI * n_rot);

  //  std::cout << "dirR: rad: " << phi_dir << " = " << phi_dir/M_PI << "pi deg: " <<  phi_dir/M_PI*180 << std::endl;

  w_ = phi_dir / t_traj;
}

void PointCloudProcessingNode::flyCircular()
{
  //angles have to be normed as they are used as references in a polar/cylindrical coordinate systems
  //pos/ccw direction
  if (dir_)
    T_ = 2 * M_PI / w_des_;
  t_helix_ = T_ * n_rot_des_;
  //v_z_helix_ = h_diff_ /t_helix_;

  cur_r_ = r_;
  float phi_dir = (2 * M_PI * n_rot_des_);

  //  std::cout << "dirR: rad: " << phi_dir << " = " << phi_dir/M_PI << "pi deg: " <<  phi_dir/M_PI*180 << std::endl;

  w_ = (phi_dir / t_helix_)/ 15;

  cur_phi_ = std::fmod(pre_phi_ + (w_ * trajDur()), 2 * M_PI);

  /* //neg/cw direction
  if(!dir_) {
    cur_phi_ = pre_phi_ - std::fmod(w_ * trajDur(), 2 * M_PI);
    if (std::signbit(cur_phi_)) { //true if negative
      cur_phi_ = (2 * M_PI) + cur_phi_;
    }
  }*/

  //TODO: get rid of/resp completely swap to cylindrical coordinate system
  toCarthesian();
}

void PointCloudProcessingNode::toCarthesian()
{

  cur_x_ = (cur_r_ * std::cos(cur_phi_));
  cur_y_ = (cur_r_ * std::sin(cur_phi_));
}

void PointCloudProcessingNode::updatePosMsg()
{
  pos_msg_.header.stamp = ros::Time::now();
  pos_msg_.header.frame_id = set_point_frame;

  pos_msg_.x = cur_x_;
  pos_msg_.y = cur_y_;
  pos_msg_.z = cur_z_;
  pos_msg_.roll = cur_roll_;
  pos_msg_.pitch = 0.0; //-(tz*2.0);
  pos_msg_.yaw = std::fmod((cur_phi_ / M_PI * 180) + 180, 360.0);
}

void PointCloudProcessingNode::setPrePose()
{
  pre_x_ = cur_x_;
  pre_y_ = cur_y_;
  pre_z_ = cur_z_;
  pre_roll_ = cur_roll_;
  pre_pitch_ = cur_pitch_;
  pre_yaw_ = cur_yaw_;
  pre_r_ = cur_r_;
  pre_phi_ = cur_phi_;
}

void PointCloudProcessingNode::reInitialize()
{
  t_start_ = t_start_traj_ = ros::Time::now();
  cur_fl_state_ = INITPOS;
  prev_fl_state_ = INITPOS;
  cur_x_ = -3.0;
  cur_y_ = cur_z_ = cur_roll_ = cur_yaw_ = 0.0;
  cur_pitch_ = 0.0; //-(tz*2.0);
  pre_x_ = pre_y_ = pre_z_ = pre_roll_ = pre_yaw_ = 0.0;
  pre_pitch_ = 0.0; //-(tz*2.0);
  cur_r_ = 3.0;
  cur_phi_ = M_PI;
  pre_r_ = 3.0;
  pre_phi_ = M_PI;
  w_ = 0.0;
}

void PointCloudProcessingNode::OrbslamFilterCallback(const PointCloud::ConstPtr &input_cloud)
{

  pcl::PointCloud<pcl::PointXYZ>::Ptr points_to_inspect(new pcl::PointCloud<pcl::PointXYZ>);
  points_to_inspect->header.frame_id = "world";
  pcl::PointCloud<pcl::PointXYZ>::Ptr points_on_surface(new pcl::PointCloud<pcl::PointXYZ>);
  points_on_surface->header.frame_id = "world";
  pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<control_api_waypoint> waypointList;
  control_api_waypoint newwaypoint;

  //PointCloud::Ptr cropped_cloud = PointCloud::Ptr(new PointCloud);
  cropped_cloud->header.frame_id = "world";
  //sensor_msgs::PointCloud2 msg_output;

  //TODO: set to common bounds
  pcl::CropBox<Point> boxFilter;
  boxFilter.setMin(Eigen::Vector4f(-1.1, -1.1,
                                   -0.1, 1.0)); // -0.1 due inaccuracy of ORB-SLAM2
  //  boxFilter.setMin(Eigen::Vector4f(obj_center_x_ - obj_dim_x_, obj_center_y_ - obj_center_y_,
  //                                   -0.1, 1.0)); //z -0.1 due inaccuracy of ORB-SLAM2
  boxFilter.setMax(Eigen::Vector4f(1.1, +1.1,
                                   8.1, 1.0)); //z +0.1 due inaccuracy of ORB-SLAM2
  //  boxFilter.setMax(Eigen::Vector4f(obj_center_x_ + obj_dim_x_,  obj_center_y_ + obj_center_y_,
  //                                   3.1, 1.0)); //z +0.1 due inaccuracy of ORB-SLAM2
  boxFilter.setInputCloud(input_cloud);
  boxFilter.filter(*cropped_cloud);

  pub_filterCloud.publish(*cropped_cloud);
  msg.data = cropped_cloud->width;

  NoP_pub.publish(msg);

  pcl::PointXYZ pt_voxel_center;
  float resolution = 0.8f;

  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);

  octree.setInputCloud(cropped_cloud);
  octree.addPointsFromInputCloud();

  pcl::octree::OctreePointCloudPointVector<pcl::PointXYZ>::LeafNodeIterator it;
  const pcl::octree::OctreePointCloudPointVector<pcl::PointXYZ>::LeafNodeIterator it_end = octree.leaf_end();

  pcl::PointXYZ searchPoint;
  for (auto it = octree.leaf_begin(); it != it_end; ++it)
  {
    Eigen::Vector3f voxel_min, voxel_max;
    octree.getVoxelBounds(it, voxel_min, voxel_max);
    pt_voxel_center.x = (voxel_min.x() + voxel_max.x()) / 2.0f;
    pt_voxel_center.y = (voxel_min.y() + voxel_max.y()) / 2.0f;
    pt_voxel_center.z = (voxel_min.z() + voxel_max.z()) / 2.0f;

    searchPoint.x = pt_voxel_center.x;
    searchPoint.y = pt_voxel_center.y;
    searchPoint.z = pt_voxel_center.z;

    newwaypoint.x = searchPoint.x;
    newwaypoint.y = searchPoint.y;
    newwaypoint.z = searchPoint.z;

    std::vector<int> pointIdxVec;

    beta = atan2(ty, tx);

    if (beta < 0.0)
    {
      beta += 2 * M_PI; //360.0;
    }

    theta = atan2(searchPoint.y, searchPoint.x); //*(180/M_PI);

    if (theta < 0.0)
    {
      theta += 2 * M_PI; //360.0;
    }

    int count = 0;

    if (octree.voxelSearch(searchPoint, pointIdxVec))
    {
      if ((pt_voxel_center.z < 14.8) && (pt_voxel_center.z > 0.2))
      {
        if ((pointIdxVec.size() > 1) && (pointIdxVec.size() < 15))
        {
          points_on_surface->points.push_back(pcl::PointXYZ(searchPoint.x, searchPoint.y, searchPoint.z));
          pub_onSurface.publish(*points_on_surface);
          dfc = sqrt(pow((searchPoint.x - 0.0), 2) + pow((searchPoint.y - 0.0), 2));
          if (dfc < 3.0)
          {
            float ix = 3 - dfc;
            dfc = dfc + ix;
          }
          else if (dfc > 3.0)
          {
            float dx = dfc - 3.0;
            dfc = dfc - dx;
          }
          rp_ = 3.0;

          std::cout << "Theta before conversion" << theta << std::endl;

          if ((theta > 0) && (theta < M_PI))
          {
            searchPoint.x = dfc * std::cos(theta);
            searchPoint.y = dfc * std::sin(theta);
            searchPoint.z = searchPoint.z + 1.5;
          }
          else if ((theta > M_PI) && (theta < (2 * M_PI)))
          {
            searchPoint.x = dfc * std::cos(theta);
            searchPoint.y = dfc * std::sin(theta);
            searchPoint.z = searchPoint.z + 1.0;
          }

          theta = atan2((searchPoint.y - 0.0), (searchPoint.x - 0.0));

          std::cout << "Theta after conversion" << theta << std::endl;

          dfc = sqrt(pow((searchPoint.x - 0.0), 2) + pow((searchPoint.y - 0.0), 2));

          std::cout << "Distance from centre" << dfc << std::endl;

          points_to_inspect->points.push_back(pcl::PointXYZ(searchPoint.x, searchPoint.y, searchPoint.z));

          if (go_to_sleep)
          {
            str.data = "true";
            pub_Helixcomplete.publish(str);

            pub.publish(*points_to_inspect);

            std::cout << "helix_state_complete" << std::endl;
            ros::Duration(20.0).sleep();
          }
        }
      }
    }
  }
}