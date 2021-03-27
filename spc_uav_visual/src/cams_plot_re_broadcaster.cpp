/*
 * controller_node.cpp
 *
 *  Created on: Feb 11, 2019
 *      Author: Patrick Radl
 *
 *
 * The purpose of this simple node is to listen to the for plotting relevant
 * transforms and publish them on own topics.
 * This is necessary as multiple tf publishers cause the transform vector index
 * to not always point to the same frame.
 */


#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>


void initParameters();

std::string node_name = "cams_plot_re_broadcaster";
std::string estimated_camera_frame_id, physical_camera_frame_id, physical_world_frame_id;
std::string est_cam_topic, phys_cam_topic, error_cam_topic;
int msg_buffer_size;

// calculatres translation error/offset between two transforms - rotation not taken into account yet and header mostly copied from t1
void calcTranError(geometry_msgs::TransformStamped& t1, geometry_msgs::TransformStamped& t2, geometry_msgs::TransformStamped& result)
{
  static int seq = 0;

  result.header.frame_id = "do-not-use";
  result.header.seq = seq;
  result.header.stamp = t1.header.stamp;
  result.child_frame_id = "do-not-use";

  result.transform.translation.x = fabs(t1.transform.translation.x - t2.transform.translation.x) + fabs(t1.transform.translation.y - t2.transform.translation.y);
  result.transform.translation.y = 0;//fabs(t1.transform.translation.y - t2.transform.translation.y);
  result.transform.translation.z = fabs(t1.transform.translation.z - t2.transform.translation.z);

//  result.transform.translation.x = fabs(t1.transform.translation.x - t2.transform.translation.x);
//  result.transform.translation.y = fabs(t1.transform.translation.y - t2.transform.translation.y);
//  result.transform.translation.z = fabs(t1.transform.translation.z - t2.transform.translation.z);

  seq++;
}

int main (int argc, char** argv)
{
  ros::init(argc, argv, node_name);
  ros::NodeHandle n;

  tf2_ros::Buffer est_cam_b, phys_cam_b;
  geometry_msgs::TransformStamped est_cam_t, phys_cam_t, error_cam_t;
  tf2_ros::TransformListener est_cam_list(est_cam_b);
  tf2_ros::TransformListener phys_cam_list(phys_cam_b);


  n.param<std::string>(node_name+"/estimated_camera_frame_id", estimated_camera_frame_id, "estimated_camera_link");
  n.param<std::string>(node_name+"/physical_camera_frame_id", physical_camera_frame_id, "physical_camera_link");
  n.param<std::string>(node_name+"/physical_world_frame_id", physical_world_frame_id, "world");
  n.param<std::string>(node_name+"/est_cam_topic", est_cam_topic, "est_cam_tf");
  n.param<std::string>(node_name+"/phys_cam_topic", phys_cam_topic, "phys_cam_tf");
  n.param<std::string>(node_name+"/error_cam_topic", error_cam_topic, "error_cam_tf");
  n.param(node_name+"/msg_buffer_size", msg_buffer_size, 500);

  ros::Publisher error_cam_pub = n.advertise<geometry_msgs::TransformStamped>(error_cam_topic, msg_buffer_size);
  ros::Publisher est_cam_pub = n.advertise<geometry_msgs::TransformStamped>(est_cam_topic, msg_buffer_size);
  ros::Publisher phys_cam_pub = n.advertise<geometry_msgs::TransformStamped>(phys_cam_topic, msg_buffer_size);




  ros::Rate rate(10.0);
  while (ros::ok()){
    try{
      est_cam_t = est_cam_b.lookupTransform(physical_world_frame_id, estimated_camera_frame_id, ros::Time(0), ros::Duration(0.1));
    }
    catch (tf2::TransformException &e) {
      ROS_WARN("%s",e.what());
      continue;
    }
    try{
      phys_cam_t = phys_cam_b.lookupTransform(physical_world_frame_id, physical_camera_frame_id, est_cam_t.header.stamp, ros::Duration(0.1));
    }
    catch (tf2::TransformException &e) {
      ROS_WARN("%s",e.what());
      continue;
    }

    calcTranError(phys_cam_t, est_cam_t, error_cam_t);

    est_cam_pub.publish(est_cam_t);
    phys_cam_pub.publish(phys_cam_t);
    error_cam_pub.publish(error_cam_t);





    rate.sleep();
  }

  return 0;
}
