/*
 * geom_impedance_controller_node.cpp
 *
 *  Created on: June, 2018
 *      Author: ramy
 */

#include "spc_uav_control/geom_impedance_controller_node.h"

namespace impedance_controller {

GeomImpedanceControllerNode::GeomImpedanceControllerNode()
    : motion_nh_("~/motion"),
      motionDServer(motion_nh_) {
  // Load from the ROS Parameter Server the controller parameters
  loadImpedanceROSParameters();

  // Initialize Dynamic Reconfigure
  motionDServer.setCallback(
      boost::bind(&GeomImpedanceControllerNode::dynamicReconfigureImpedanceCallback, this, _1, _2));
}

void GeomImpedanceControllerNode::loadImpedanceROSParameters() {
  ros::NodeHandle node_priv("~");

  float dummy;
  if (!node_priv.getParam("gravity_constant", dummy)) {
    ROS_ERROR("Could not find gravitational constant parameter!");
  }
  controller_.setGravity(dummy);

  node_priv.param<float>("x_pos_end_effector", dummy, 0.0);

  setPosXEndEffector(dummy);
}

void GeomImpedanceControllerNode::dynamicReconfigureImpedanceCallback(
    const spc_uav_control::Impedance_Control_GainsConfig &config, const uint32_t level) {
  controller_.setMass(config.mass);
  controller_.setInertiaDiag(Eigen::Vector3d(config.J_xx, config.J_yy, config.J_zz));
  controller_.setVelocityGain(Eigen::Vector3d(config.K_v_x, config.K_v_y, config.K_v_z));
  controller_.setAngularRateGain(Eigen::Vector3d(config.K_w_x, config.K_w_y, config.K_w_z));

  controller_.setK_p_r(Eigen::Vector3d(config.c_KP_r_xx, config.c_KP_r_yy, config.c_KP_r_zz));
  controller_.setK_p_t(Eigen::Vector3d(config.c_KP_t_xx, config.c_KP_t_yy, config.c_KP_t_zz));

  controller_.setTrackEndEffector(config.track_end_effector);

  std::string sep;
  Eigen::Vector3d px4AttitudeLimits(getCmdRollMomentLimit(), getCmdPitchMomentLimit(), getCmdYawMomentLimit());

  if (isUseMavlinkInterface()) {

    Eigen::Vector3d px4Gains(config.K_w_x_px4, config.K_w_y_px4, config.K_w_z_px4);

    setPx4Gains(px4Gains);

    controller_.setOnboardAngularRateGain(px4Gains.cwiseProduct(px4AttitudeLimits));
  } else {
    setPx4Gains(Eigen::Vector3d::Zero(3));
    controller_.setOnboardAngularRateGain(Eigen::Vector3d::Zero(3));

    Eigen::Vector3d rate_Gain = controller_.getAngularRateGain();
    rate_Gain = rate_Gain.cwiseQuotient(px4AttitudeLimits);

    sep = "\n----------Equivalent PX4 Rate Gains-----------\n";
    std::cout << sep << rate_Gain.transpose() << sep;
  }

  if (controller_.isTrackEndEffector()) {
    double xE2 = std::pow(getPosXEndEffector(),2);
    double cy = config.J_yy/config.mass;
    double cz = config.J_zz/config.mass;


    sep = "\n----Recommend Tracking EE Rotational Gains-------\n";
    std::cout << sep << "Kr_y:" << config.c_KP_r_yy + xE2 * config.c_KP_t_zz*cy
        << " Kr_z:"<< config.c_KP_r_zz + xE2 * config.c_KP_t_yy*cz
        << " Kw_y:"<< config.K_w_y + xE2 * config.K_v_z*cy
        << " Kw_z:"<< config.K_w_z + xE2 * config.K_v_y*cz<< sep;
  }

  sep = "\n----------Natural Frequencies-----------\n";
  std::cout << sep << controller_.computeNaturalFreq() << sep;

  sep = "\n----------Damping Coefficients-----------\n";
  std::cout << sep << controller_.computeDampingCoeff() << sep;

  ROS_INFO(" \n Gains changed in Impedance Controller!");
}

void GeomImpedanceControllerNode::sendControlWrench() {

  if (isControllerActive())
    controller_.setControllerActive(true);
  else
    controller_.setControllerActive(false);
  // Compute elapsed time
  controller_.setDtI(computeElapsedTime());

  // Calculate control wrench
  Eigen::VectorXd command_wrench;
  controller_.calculateCommandWrench(&command_wrench, _H_B_E, _H_I_B, _T_B, getH_I_C(), getT_C(), getTdot_C());

  // Either Use SITL interface OR RotorS interface
  publishSetPoints(command_wrench);
  // Publish Control Signals
  publishControlSignals(command_wrench);
}
}

int main(int argc, char **argv) {

  // Initialize ROS Node
  ros::init(argc, argv, "geometric_impedance_controller_node");
  impedance_controller::GeomImpedanceControllerNode impedance_controller_node;

  // Update Set Point + send and capture control signals at 100 Hz
  ros::Rate r(100);  //TODO: CHANGE TO STATIC ROSPARAM
  while (ros::ok()) {
    ros::spinOnce();
    impedance_controller_node.updateGUISetpoint();
    impedance_controller_node.sendControlWrench();
    r.sleep();
  }

  return 0;
}
