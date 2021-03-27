/*
 * geometric_SO3_controller_node.cpp
 *
 *  Created on: Jun 26, 2018
 *      Author: ramy
 */

#include "spc_uav_control/geometric_SO3_controller_node.h"


namespace motion_controller{

GeometricSO3ControllerNode::GeometricSO3ControllerNode():
    motion_nh_("~/motion"),
    motionDServer(motion_nh_)
{
  // Load from the ROS Parameter Server the controller parameters
  loadSO3ROSParameters();

  // Initialize Dynamic Reconfigure
  motionDServer.setCallback(boost::bind(&GeometricSO3ControllerNode::dynamicReconfigureSO3Callback, this, _1, _2));
}

void GeometricSO3ControllerNode::loadSO3ROSParameters() {
  ros::NodeHandle node_priv("~");

  float dummy;
  if (!node_priv.getParam("gravity_constant", dummy)) {
    ROS_ERROR("Could not find gravitational constant parameter!");
  }
  controller_.setGravity(dummy);
}


void GeometricSO3ControllerNode::dynamicReconfigureSO3Callback(const spc_uav_control::SO3_Control_GainsConfig &config,
                                                               const uint32_t level) {
  controller_.setMass(config.mass);
  controller_.setInertiaDiag(Eigen::Vector3d(config.J_xx, config.J_yy, config.J_zz));
  controller_.setPositionGain(Eigen::Vector3d(config.K_p_x, config.K_p_y, config.K_p_z));
  controller_.setVelocityGain(Eigen::Vector3d(config.K_v_x, config.K_v_y, config.K_v_z));
  controller_.setAttitudeGain(Eigen::Vector3d(config.K_R_x, config.K_R_y, config.K_R_z));
  controller_.setAngularRateGain(Eigen::Vector3d(config.K_w_x, config.K_w_y, config.K_w_z));
  Eigen::Vector3d px4Gains(config.K_w_x_px4, config.K_w_y_px4, config.K_w_z_px4);
  setPx4Gains(px4Gains);


  ROS_INFO("Gains changed in SO3 Controller!");
}

void GeometricSO3ControllerNode::sendControlWrench() {

  if (isControllerActive()) controller_.setControllerActive(true);
  else controller_.setControllerActive(false);
  // Compute elapsed time
  controller_.setDtI(computeElapsedTime());

  // Calculate control wrench
  Eigen::VectorXd command_wrench;
  controller_.calculateCommandWrench(&command_wrench, _H_I_B, _T_B,
                                     getH_I_C(), getT_C(), getTdot_C());

  // Either Use SITL interface OR RotorS interface
  publishSetPoints(command_wrench);
  // Publish Control Signals
  publishControlSignals(command_wrench);
}

}


int main(int argc, char **argv) {

  // Initialize ROS Node
  ros::init(argc, argv, "geometric_SO3_controller_node");
  motion_controller::GeometricSO3ControllerNode motion_controller_node;

  // Update Set Point + send and capture control signals at 100 Hz
  ros::Rate r(100);  //TODO: CHANGE TO STATIC ROSPARAM
  while (ros::ok()) {
    ros::spinOnce();
    motion_controller_node.updateGUISetpoint();
    motion_controller_node.sendControlWrench();
    r.sleep();
  }

  return 0;
}

