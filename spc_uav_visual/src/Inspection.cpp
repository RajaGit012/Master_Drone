#include "spc_uav_visual/pointcloud_processing_node.h"

#include <boost/foreach.hpp>
#include <vector>

#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <spc_uav_visual/my_msg.h>

#include <bits/stdc++.h>
#include <iomanip>
#include <math.h>

#include <spc_uav_visual/cloud_segment.h>

#include <cmath>

//for logfile creation
#include <sstream>
#include <chrono>
#include <ctime>

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

#include "geometry_msgs/Point.h"
#include <spc_uav_visual/Inspection.h>
#include <cmath>

//for logfile creation
#include <sstream>
#include <chrono>
#include <ctime>
#include <pcl/point_cloud.h>

#include <std_msgs/Int32.h>

#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree_container.h>
#include <pcl/octree/octree_pointcloud_pointvector.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/mls.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int64.h>

#include <iostream>
#include <ctime>
#include <string>
#include <ctime>
#include "std_msgs/String.h"

enum InspState
{
   INSPEC,
   WAIT,
   END_RECONST
};
InspState cur_pt_state_;
InspState prev_pt_state_;
InspState End_pt_state_;

//Get the GUI flag to check if the Inspection optioin is selected 
void InspModeGuiCallback(const std_msgs::Bool::ConstPtr &msg)
{
   static bool prev_flag = false;

   prev_flag = gui_flag_;
   gui_flag_ = msg->data;

   if (!prev_flag && gui_flag_)
   {
      reInitialize();
      Insp_ = true;
      ROS_INFO("PCProc: GUI changed mode to Inspoverride");
   }
   else if (prev_flag && !gui_flag_)
   {
      reInitialize();
      Insp_ = false;
      ROS_INFO("PCProc: GUI stopped Inspoverride");
      //TODO: reset behaviour would have to be done in gui
   }
}

//Receive the Helix_complete status from node Helix_Status
void HelixStatusCallback(const std_msgs::String::ConstPtr &msg)
{
   Helix_complete = msg->data.c_str();
}

// if "helix complete" status is true and Inspect GUI is pressed start inspection
void InspecTimerCallback(const ros::TimerEvent &)
{
   if ((Helix_complete == "true") && Insp_)
   {
      Inspection();
      updatePosMsg();
      set_p_insp_pub_.publish(Insp_pos_msg_);
   }
}

void ORBPointsCallback(const sensor_msgs::PointCloud2ConstPtr &ORB_cloud)
{

   msg.data = ORB_cloud->width;
   // Orb_points = msg.data;
   rawpoint_pub.publish(msg);
}

void EndReconstruct(const sensor_msgs::PointCloud2ConstPtr &CroppedCloud)
{
   if (gui_flag_)
   {
      int new_NoP = CroppedCloud->width;

      if ((new_NoP - old_NoP) > 500)
      {
         count--;
         std::cout << "CONTINUE" << count << std::endl;
      }
      else
      {
         count++;
         if (count == 2)
         {
            End_pt_state_ == END_RECONST;
            std::cout << "ENDRECONSTRUCTION" << std::endl;
         }
         std::cout << "STOP" << count << std::endl;
      }

      old_NoP = new_NoP;
   }
}

void waitCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
   tx = msg->pose.pose.position.x;
   ty = msg->pose.pose.position.y;
   tz = msg->pose.pose.position.z;
   tpitch = msg->pose.pose.orientation.z;
}

void InspecPointsCallback(const sensor_msgs::PointCloud2ConstPtr &inspec_cloud)
{
   control_api_waypoint newwaypoint;
   for (sensor_msgs::PointCloud2ConstIterator<float> it(*inspec_cloud, "x"); it != it.end(); ++it)
   {
      // TODO: do something with the values of x, y, z
      Ix = it[0];
      Iy = it[1];
      Iz = it[2];
      newwaypoint.x = Ix;
      newwaypoint.y = Iy;
      newwaypoint.z = Iz;
      waypointList.push_back(newwaypoint);
   }
}

double trajDur()
{
   return (ros::Time::now() - t_start_traj_).toSec();
}

void Inspection()
{
   if ((cur_pt_state_ == INSPEC) && (End_pt_state_ != END_RECONST))
   {
      //std::cout << "Inspection"<< std::endl;

      int n = waypointList.size();
      Wx = waypointList[n - 1].x;
      Wy = waypointList[n - 1].y;
      Wz = waypointList[n - 1].z;
      dist = sqrt(pow((Wx - tx), 2.0) + pow((Wy - ty), 2.0) + pow((Wz - tz), 2.0));
      float start_angle = (atan2(ty, tx));
      start_angle = floorf(start_angle * 100) / 100;
      if (start_angle < 0)
      {
         start_angle += 2 * M_PI;
      }
      float dest_angle = (atan2(Wy, Wx));
      dest_angle = floorf(dest_angle * 100) / 100;
      if (dest_angle < 0)
      {
         dest_angle += 2 * M_PI;
      }
      if ((dist > 0.5) && (start_angle != dest_angle))
      {
         flytopoints(start_angle, dest_angle);
      }
      else
      {
         cur_pt_state_ = WAIT;
         prev_pt_state_ = INSPEC;
         transition();
      }
   }
   else if ((cur_pt_state_ == WAIT) && (End_pt_state_ != END_RECONST))
   {
      if (ros::Time::now().toSec() < (t_start_traj_.toSec() + t_wait))
      {
         if (dist > 0.5)
         {
            if (tz > (Iz + 0.4))
            {
               cur_z_ = -(0.2 * trajDur()) + pre_z_;
            }
            else if (tz < (Iz - 0.4))
            {
               cur_z_ = (0.2 * trajDur()) + pre_z_;
            }
         }
         else if (dist < 0.5)
         {
            if (tz > Iz)
            {
               cur_z_ = -(0.004 * trajDur()) + pre_z_;
            }
            else if (tz < Iz)
            {
               cur_z_ = (0.004 * trajDur()) + pre_z_;
            }
         }
      }
      else
      {
         cur_pt_state_ = INSPEC;
         prev_pt_state_ = WAIT;
         counter++;
         transition();
      }
   }
   else if ((cur_pt_state_ == WAIT) && (End_pt_state_ == END_RECONST))
   {
      if (ros::Time::now().toSec() < (t_start_traj_.toSec() + t_end))
      {
         cur_z_ = -(0.4 * trajDur()) + pre_z_;
      }
   }
}

void flytopoints(float start_angle, float dest_angle)
{
  if((start_angle - dest_angle) > 0)
  {
     if((start_angle - dest_angle) < M_PI)
     {
        //clock
        direction = true;
      
        if(tz < Iz)
       {
           if(start_angle != dest_angle)
          {
             cur_phi_ = pre_phi_ - (0.15 * trajDur());
             cur_x_ = (3.0 * std::cos(cur_phi_));
             cur_y_ = (3.0 * std::sin(cur_phi_));
             cur_z_ = (0.02 * trajDur()) + pre_z_;
            
          }
           else if ((start_angle > dest_angle + 0.2)&&(start_angle < dest_angle - 0.2))
          {
             cur_z_ = (0.02 * trajDur()) + pre_z_;
          
          }
       }
            else if ( tz > Iz)
        {
            if (start_angle != dest_angle)
         {
            cur_phi_ = pre_phi_ - (0.15 * trajDur());
            cur_x_ = (3.0 * std::cos(cur_phi_));
            cur_y_ = (3.0 * std::sin(cur_phi_));
            cur_z_ =  -(0.02 * trajDur()) + pre_z_;
          
         }
            else if ((start_angle > dest_angle + 0.2)&&(start_angle < dest_angle - 0.2))
         {
            cur_z_ = -(0.02 * trajDur()) + pre_z_;
          
         }
        }
        
      }
     else if((start_angle - dest_angle) > M_PI)
     {
        //anti
        direction = true;
      
        if(tz < Iz)
       {
           if(start_angle != dest_angle)
          {
             cur_phi_ = pre_phi_ + (0.15 * trajDur());
             cur_x_ = (3.0 * std::cos(cur_phi_));
             cur_y_ = (3.0 * std::sin(cur_phi_));
             cur_z_ = (0.02 * trajDur()) + pre_z_;
             
          }
           else if ((start_angle > dest_angle + 0.2)&&(start_angle < dest_angle - 0.2))
          {
             cur_z_ = (0.02 * trajDur()) + pre_z_;
             
          }
       }
            else if ( tz > Iz)
        {
            if (start_angle != dest_angle)
         {
            cur_phi_ = pre_phi_ + (0.15 * trajDur());
            cur_x_ = (3.0 * std::cos(cur_phi_));
            cur_y_ = (3.0 * std::sin(cur_phi_));
            cur_z_ =  -(0.02 * trajDur()) + pre_z_;
            
         }
            else if ((start_angle > dest_angle + 0.2)&&(start_angle < dest_angle - 0.2))
         {
            cur_z_ = -(0.02 * trajDur()) + pre_z_;
      
         }
        }
     }
  }
  else if ((start_angle - dest_angle) < 0)
  {
     if((start_angle - dest_angle) < -(M_PI))
     {
        //anti
        direction = true;
      
        if(tz < Iz)
       {
           if(start_angle != dest_angle)
          {
             cur_phi_ = pre_phi_ - (0.15 * trajDur());
             cur_x_ = (3.0 * std::cos(cur_phi_));
             cur_y_ = (3.0 * std::sin(cur_phi_));
             cur_z_ = (0.02 * trajDur()) + pre_z_;
             
          }
           else if ((start_angle > dest_angle + 0.2)&&(start_angle < dest_angle - 0.2))
          {
             cur_z_ = (0.02 * trajDur()) + pre_z_;
             
          }
       }
            else if ( tz > Iz)
        {
            if (start_angle != dest_angle)
         {
            cur_phi_ = pre_phi_ - (0.15 * trajDur());
            cur_x_ = (3.0 * std::cos(cur_phi_));
            cur_y_ = (3.0 * std::sin(cur_phi_));
            cur_z_ =  -(0.02 * trajDur()) + pre_z_;
            
         }
            else if ((start_angle > dest_angle + 0.2)&&(start_angle < dest_angle - 0.2))
         {
            cur_z_ = -(0.02 * trajDur()) + pre_z_;
        
         }
        }
     }
     else if((start_angle - dest_angle) > -(M_PI))
     {
       //clock
       direction = true;
      
        if(tz < Iz)
       {
           if(start_angle != dest_angle)
          {
             cur_phi_ = pre_phi_ + (0.15 * trajDur());
             cur_x_ = (3.0 * std::cos(cur_phi_));
             cur_y_ = (3.0 * std::sin(cur_phi_));
             cur_z_ = (0.02 * trajDur()) + pre_z_;
             
          }
           else if ((start_angle > dest_angle + 0.2)&&(start_angle < dest_angle - 0.2))
          {
             cur_z_ = (0.02 * trajDur()) + pre_z_;
            
          }
       }
            else if ( tz > Iz)
        {
            if (start_angle != dest_angle)
         {
            cur_phi_ = pre_phi_ + (0.15 * trajDur());
            cur_x_ = (3.0 * std::cos(cur_phi_));
            cur_y_ = (3.0 * std::sin(cur_phi_));
            cur_z_ =  -(0.02 * trajDur()) + pre_z_;
         
         }
            else if ((start_angle > dest_angle + 0.2)&&(start_angle < dest_angle - 0.2))
         {
            cur_z_ = -(0.02 * trajDur()) + pre_z_;
           
         }
        } 
     }

  }
} 

void transition()
{
   setPrePose();
   t_start_traj_ = ros::Time::now();
   transition_ = true;
}

void updatePosMsg()
{
   Insp_pos_msg_.header.stamp = ros::Time::now();
   Insp_pos_msg_.header.frame_id = "world";
   Insp_pos_msg_.x = cur_x_;
   Insp_pos_msg_.y = cur_y_;
   Insp_pos_msg_.z = cur_z_;
   Insp_pos_msg_.roll = 0.0;
   if (tz < Iz)
   {
      Insp_pos_msg_.pitch = 10.0; //(tz*2);
   }
   else if (tz > Iz)
   {
      Insp_pos_msg_.pitch = 10.0;
   }
   Insp_pos_msg_.yaw = std::fmod((cur_phi_ / M_PI * 180) + 180, 360.0);
}

void setPrePose()
{
   pre_x_ = tx;
   pre_y_ = ty;
   pre_z_ = tz;
   pre_roll_ = cur_roll_;
   pre_pitch_ = cur_pitch_;
   pre_yaw_ = cur_yaw_;
   //pre_r_ = cur_r_;
   pre_phi_ = cur_phi_;
}

void reInitialize()
{
   t_start_traj_ = ros::Time::now();
   cur_pt_state_ = INSPEC;
   prev_pt_state_ = INSPEC;
   End_pt_state_ != END_RECONST;
   transition_ = false;
   cur_x_ = tx;
   cur_y_ = ty;
   cur_z_ = tz;
   cur_roll_ = 0.0;
   cur_pitch_ = 10.0; //(tz*1.5);
   cur_yaw_ = tpitch;
   pre_x_ = pre_y_ = pre_roll_ = pre_yaw_ = 0.0;
   pre_pitch_ = 10.0; //(tz*1.5);
   pre_z_ = tz;
   //cur_r_ = 3.0;
   cur_phi_ = (atan2(ty, tx));
   //pre_r_ = 3.0;
   pre_phi_ = cur_phi_;
   //w_ = 0.0;
}

int main(int argc, char **argv)
{

   ros::init(argc, argv, "Inspec");

   ros::NodeHandle n;

   ros::Subscriber sub_ov = n.subscribe("gui/InspOvRide", 1, InspModeGuiCallback);

   ros::Subscriber ORB_ov = n.subscribe("/orb_slam2_stereo/map_points", 1, ORBPointsCallback);

   ros::Subscriber Insp_ov = n.subscribe("/points_to_inspect", 1, InspecPointsCallback);

   ros::Subscriber Delay_ov = n.subscribe("/ground_truth/odometry", 1, waitCallback);

   ros::Subscriber sub_NoP = n.subscribe("/cropped_cloud", 1000, EndReconstruct);

   ros::Subscriber sub = n.subscribe("/Helix_Status", 1000, HelixStatusCallback);

   ros::Timer timer = n.createTimer(ros::Duration(0.001), InspecTimerCallback);

   set_p_insp_pub_ = n.advertise<spc_uav_visual::InspPos>("/Insp_p", 1);

   rawpoint_pub = n.advertise<std_msgs::Int64>("/Raw_Points", 1000);

   ros::spin();
}
