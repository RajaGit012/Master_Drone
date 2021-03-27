/*
 * geometric_SE3_controller_node.cpp
 *
 *  Created on: Jan 19, 2018
 *      Author: ramy
 *  Refactored on: May 2018
 *      Author: Asem
 */

#include "spc_uav_control/geometric_SE3_controller_node.h"

namespace motion_controller{

GeometricSE3ControllerNode::GeometricSE3ControllerNode():
    motion_nh_("~/motion"),
    motionDServer(motion_nh_)
{
  // Load from the ROS Parameter Server the controller parameters
  loadSE3ROSParameters();

  // Initialize Dynamic Reconfigure
  motionDServer.setCallback(boost::bind(&GeometricSE3ControllerNode::dynamicReconfigureSE3Callback, this, _1, _2));
}


void GeometricSE3ControllerNode::loadSE3ROSParameters() {
  ros::NodeHandle node_priv("~");

  int disturbance_observer_type;
  node_priv.param<int>("disturbance_observer_type", disturbance_observer_type, NODISTURBANCEOBSERVER);
  controller_.setDisturbanceObserverType(disturbance_observer_type);

  float dummy;
  if (!node_priv.getParam("gravity_constant", dummy)) {
    ROS_ERROR("Could not find gravitational constant parameter!");
  }
  controller_.setGravity(dummy);
}


void GeometricSE3ControllerNode::dynamicReconfigureSE3Callback(const spc_uav_control::SE3_Control_GainsConfig &config,
                                                               const uint32_t level) {
  controller_.setMass(config.mass);
  controller_.setInertiaDiag(Eigen::Vector3d(config.J_xx, config.J_yy, config.J_zz));
  controller_.setPositionGain(Eigen::Vector3d(config.K_p_x, config.K_p_y, config.K_p_z));
  controller_.setVelocityGain(Eigen::Vector3d(config.K_v_x, config.K_v_y, config.K_v_z));
  controller_.setAttitudeGain(Eigen::Vector3d(config.K_R_x, config.K_R_y, config.K_R_z));
  controller_.setAngularRateGain(Eigen::Vector3d(config.K_w_x, config.K_w_y, config.K_w_z));
  setPx4Gains(
      Eigen::Vector3d(1.0 / (config.K_w_x_px4), 1.0 / (config.K_w_y_px4), 1.0 / (config.K_w_z_px4)));
  controller_.setDistObsGain(config.K_0_t, config.K_0_r, config.K_1_t, config.K_1_r);
  controller_.setDoOn(config.DO_ON);

  ROS_INFO("Gains changed in SE3 Controller!");
}

void GeometricSE3ControllerNode::sendControlWrench() {

  if (isControllerActive()) controller_.setControllerActive(true);
  else controller_.setControllerActive(false);
  // Compute elapsed time
  controller_.setDtI(computeElapsedTime());

  // Calculate control wrench
  Eigen::VectorXd command_wrench;
  controller_.calculateCommandWrench(&command_wrench,_H_B_E, _H_I_B, _T_B,
                                     getH_I_C(), getT_C(), getTdot_C());

  // Either Use SITL interface OR RotorS interface
  publishSetPoints(command_wrench);
  // Publish Control Signals
  publishControlSignals(command_wrench);
}
}

int main(int argc, char **argv) {

  // Initialize ROS Node
  ros::init(argc, argv, "geometric_SE3_controller_node");
  motion_controller::GeometricSE3ControllerNode motion_controller_node;

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
