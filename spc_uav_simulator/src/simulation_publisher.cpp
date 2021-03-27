/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/TransformStamped.h"
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <vector>
#include <string>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

nav_msgs::Odometry current_odom_msg;
gazebo_msgs::ModelStates current_model_states;
ros::Publisher ground_truth_pub;
std::string ground_truth_frame_id, ground_truth_child_frame_id, uav_name;

void model_states_cb(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    static tf2_ros::TransformBroadcaster tfb;
    geometry_msgs::TransformStamped transformStamped;
    current_model_states = *msg;

    std::vector<std::string> modelNames = msg->name;
    for (int i = 0; i < modelNames.size(); i++) {
      if (std::string(modelNames[i]) == uav_name) {
        ROS_INFO_ONCE("Got First Gazebo Message !!");
        //current_odom_msg.pose.pose = msg->pose[i];
        //current_odom_msg.twist.twist = msg->twist[i]; // In World Frame
        transformStamped.transform.translation.x = msg->pose[i].position.x;
        transformStamped.transform.translation.y = msg->pose[i].position.y;
        transformStamped.transform.translation.z = msg->pose[i].position.z;
        transformStamped.transform.rotation.x = msg->pose[i].orientation.x;
        transformStamped.transform.rotation.y = msg->pose[i].orientation.y;
        transformStamped.transform.rotation.z = msg->pose[i].orientation.z;
        transformStamped.transform.rotation.w = msg->pose[i].orientation.w;
      }
    }

    current_odom_msg.header.stamp = transformStamped.header.stamp = ros::Time::now();

    current_odom_msg.header.frame_id = transformStamped.header.frame_id = ground_truth_frame_id;
    current_odom_msg.child_frame_id = transformStamped.child_frame_id = ground_truth_child_frame_id;

    //ground_truth_pub.publish(current_odom_msg);
    tfb.sendTransform(transformStamped);
}

// Callback function that is called when an arming event happens, and then the status of the FCU is recorded
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
  current_state = *msg;
}

ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;

void offboard_command_cb(const std_msgs::Bool msg) {
  mavros_msgs::SetMode offb_set_mode;
  mavros_msgs::CommandBool arm_cmd;

  if (msg.data) {
    offb_set_mode.request.custom_mode = "OFFBOARD";
    arm_cmd.request.value = true;

    if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
      // If changing the FCU mode was successful, inform !
      ROS_INFO("Offboard enabled");
    }

    if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
      // If arming the FCU was successful, inform !
      ROS_INFO("Vehicle armed");
    }
  } else {
    offb_set_mode.request.custom_mode = "AUTO.RTL";
    if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
      // If changing the FCU mode was successful, inform !
      ROS_INFO("Offboard disabled");
    }
  }

}

int main(int argc, char **argv) {
  // Initialize node
  ros::init(argc, argv, "simulation_publisher");
  // Create a public node handle
  ros::NodeHandle nh;
  // Create a private node handle
  ros::NodeHandle node_priv("~");

  // Create a subscriber to listen to source of arming events and handle it in callback
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

  ros::Subscriber offboard_command_sub = nh.subscribe<std_msgs::Bool>("offboardCommand", 10, offboard_command_cb);

  ros::Subscriber model_states_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10,
                                                                            model_states_cb);

  // Create a publisher that will send the position setpoint
  ground_truth_pub = nh.advertise<nav_msgs::Odometry>("ground_truth/odom", 10);

  // Create a service client that will change the arming status of the FCU
  arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  // Create a service client that will change the FCU operation mode
  set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  node_priv.param<std::string>("ground_truth_frame_id", ground_truth_frame_id, "world");
  node_priv.param<std::string>("ground_truth_child_frame_id", ground_truth_child_frame_id, "base_link");
  node_priv.param<std::string>("uav_name", uav_name, "iris");


  ros::Rate r(200);
  while (ros::ok()) {
    ros::spin();
    //ground_truth_pub.publish(current_odom_msg);
    //r.sleep();
  }

  return 0;
}
