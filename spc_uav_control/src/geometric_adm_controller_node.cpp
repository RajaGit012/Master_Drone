/*
 * geometric_SE3_controller_node.cpp
 *
 *  Created on: Jan 19, 2018
 *      Author: ramy
 *  Refactored on: May 2018
 *      Author: Asem
 */

#include "spc_uav_control/geometric_adm_controller_node.h"


GeometricAdmittanceControllerNode::GeometricAdmittanceControllerNode():
    adm_nh_("~/adm"),
    admDServer(adm_nh_)
{
  ros::NodeHandle node;
  // Initialize subscribers
  external_wrench_sub = node.subscribe("/ground_truth/end_effector/force_torque",
                                       10, &GeometricAdmittanceControllerNode::
                                       externalWrenchCallback, this);

  // Initialize Dynamic Reconfigure
  admDServer.setCallback(boost::bind(&GeometricAdmittanceControllerNode::
                                     dynamicReconfigureAdmCallback, this, _1, _2));

  adm_last_time = ros::Time::now();
}


void GeometricAdmittanceControllerNode::dynamicReconfigureAdmCallback(
    spc_uav_control::Admittance_Control_GainsConfig &config, uint32_t level) {

  interaction_controller_.setInirtialTensor(config.c_mass, Eigen::Vector3d(config.c_J_xx, config.c_J_yy, config.c_J_zz));
  interaction_controller_.setK_D(Eigen::Vector3d(config.c_KD_r_x, config.c_KD_r_y, config.c_KD_r_z),
                                 Eigen::Vector3d(config.c_KD_t_x, config.c_KD_t_y, config.c_KD_t_z));
  interaction_controller_.setK_G_r(Eigen::Vector3d(config.c_KP_r_xx, config.c_KP_r_yy, config.c_KP_r_zz));
  interaction_controller_.setK_G_t(Eigen::Vector3d(config.c_KP_t_xx, config.c_KP_t_yy, config.c_KP_t_zz));

  ROS_INFO("Gains changed in Admittance Controller Node!");

}

void GeometricAdmittanceControllerNode::externalWrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg){
  // Get wrench in sensor frame from msg and Transform it to Eigen format
  Eigen::Matrix<double, 6, 1> W_D;
  W_D.topLeftCorner(3, 1) = -Eigen::Vector3d(msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z);
  W_D.bottomLeftCorner(3, 1) = -Eigen::Vector3d(msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z);

  // Convert wrench from sensor frame to desired frame
  using namespace MathHelper;
  W_D = Adjoint(inverseH(_H_I_B*_H_B_E) * _H_I_D).transpose() * W_D;
  // W_D.setZero(); //disable filter

  //TODO: Removing this line makes the controller behave in a very strange way!!!!!
  std::cout << "------H_B_E-----\n";
  std::cout << _H_B_E << '\n';


  // Run the impedance filter
  if(isControllerActive()) interaction_controller_.setControllerActive(true);
  else interaction_controller_.setControllerActive(false);

  // calculate elapsed time
  // (cannot use the available function as this will intervene with the motion control).
  ros::Time current_time = ros::Time::now();
  ros::Duration diff = current_time - adm_last_time;
  adm_last_time = current_time;

  interaction_controller_.setDt(diff.toSec());
  interaction_controller_.impedanceFilter(_H_I_D, _T_D, _Tdot_D, W_D);
  broadcastCommandTrajectory();
}

void GeometricAdmittanceControllerNode::broadcastCommandTrajectory(){
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "world";
  transformStamped.child_frame_id = "command";

  Eigen::Vector3d pose = interaction_controller_.getH_I_C().topRightCorner(3,1);
  Eigen::Matrix3d rot = interaction_controller_.getH_I_C().topLeftCorner(3,3);
  Eigen::Quaterniond q(rot);

  transformStamped.transform.translation.x = pose[0];
  transformStamped.transform.translation.y = pose[1];
  transformStamped.transform.translation.z = pose[2];
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  br.sendTransform(transformStamped);
}


int main(int argc, char **argv) {

  // Initialize ROS Node
  ros::init(argc, argv, "geometric_admittance_controller_node");
  GeometricAdmittanceControllerNode interaction_controller_node;

  // Update Set Point + send and capture control signals at 150 Hz
  ros::Rate r(150);  //TODO: CHANGE TO STATIC ROSPARAM
  while (ros::ok()) {
    ros::spinOnce();
    interaction_controller_node.updateGUISetpoint();
    interaction_controller_node.sendControlWrench();
    r.sleep();
  }

  return 0;
}
