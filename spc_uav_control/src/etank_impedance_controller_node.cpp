/*
 * etank_impedance_controller_node.cpp
 *
 *  Created on: August, 2018
 *      Author: ramy
 */

#include "spc_uav_control/etank_impedance_controller_node.h"

namespace etank_impedance_controller {

ETankImpedanceControllerNode::ETankImpedanceControllerNode()
    : motion_nh_("~/motion"),
      motionDServer(motion_nh_) {
  // Load from the ROS Parameter Server the controller parameters
  loadImpedanceROSParameters();

  // Initialize Dynamic Reconfigure
  motionDServer.setCallback(
      boost::bind(&ETankImpedanceControllerNode::dynamicReconfigureImpedanceCallback, this, _1, _2));

  // Override control signals publisher to custom message of Energy Tank Controller
  control_signals_pub = nh_.advertise<spc_uav_comm::ControlSignals>("/etank_imp_control_signals", 1);

}

void ETankImpedanceControllerNode::loadImpedanceROSParameters() {
  ros::NodeHandle node_priv("~");

  float dummy;

  node_priv.param<float>("x_pos_end_effector", dummy, 0.0);
  setPosXEndEffector(dummy);

  node_priv.param<float>("T1_init", dummy, 10.0);
  controller_.getTank1().initTank(dummy);

  node_priv.param<float>("T2_init", dummy, 10.0);
  controller_.getTank2().initTank(dummy);

}

void ETankImpedanceControllerNode::dynamicReconfigureImpedanceCallback(
    const spc_uav_control::ET_Impedance_Control_GainsConfig &config, const uint32_t level) {

  // Upload Controller's parameters
  controller_.setMass(config.mass);
  controller_.setInertiaDiag(Eigen::Vector3d(config.J_xx, config.J_yy, config.J_zz));
  controller_.setVelocityGain(Eigen::Vector3d(config.K_v_x, config.K_v_y, config.K_v_z));
  controller_.setAngularRateGain(Eigen::Vector3d(config.K_w_x, config.K_w_y, config.K_w_z));

  controller_.setK_p_r(Eigen::Vector3d(config.c_KP_r_xx, config.c_KP_r_yy, config.c_KP_r_zz));
  controller_.setK_p_t(Eigen::Vector3d(config.c_KP_t_xx, config.c_KP_t_yy, config.c_KP_t_zz));

  // Now they are controlled from the GUI
  //controller_.enableEndEffectorTrack(config.track_end_effector);
  //controller_.enableDisturbanceObserver(config.enable_dist_obs);
  controller_.setDisturbanceObserverGains(config.K0_obs_t, config.K0_obs_r);

  controller_.enableWrenchTracking(config.enable_wrench_track);

  controller_.setCommandWrenchForceX(config.W_cmd_fx);
  controller_.setKpW(Eigen::Vector3d(config.Kp_w_tx, config.Kp_w_ty, config.Kp_w_tz),
                     Eigen::Vector3d(config.Kp_w_fx, config.Kp_w_fy, config.Kp_w_fz));
  controller_.setKiW(Eigen::Vector3d(config.Ki_w_tx, config.Ki_w_ty, config.Ki_w_tz),
                     Eigen::Vector3d(config.Ki_w_fx, config.Ki_w_fy, config.Ki_w_fz));

  controller_.setEnableEnergyTank(config.enable_energy_tank);
  controller_.getTank1().setMax(config.T1_max);
  controller_.getTank1().setMin(config.T1_min);

  setCustTrajStablznThreshold(config.cust_traj_stablzn_threshold);
  setCustTrajStablznIterations((int)config.cust_traj_stablzn_iterations);


  // Handle Onboard and Offboard implementation of Rotational Damping
  std::string sep;
  Eigen::Vector3d px4AttitudeLimits(getCmdRollMomentLimit(), getCmdPitchMomentLimit(), getCmdYawMomentLimit());
  if (isUseMavlinkInterface()) {
    // If using PX4, then the corresponding gains are set
    Eigen::Vector3d px4Gains(config.K_w_x_px4, config.K_w_y_px4, config.K_w_z_px4);
    setPx4Gains(px4Gains);
    controller_.setOnboardAngularRateGain(px4Gains.cwiseProduct(px4AttitudeLimits));
  } else {
    // If not using PX4, the px4 gains are set to zero, and the conversion from offboard gains to onboard gains is done
    setPx4Gains(Eigen::Vector3d::Zero(3));
    controller_.setOnboardAngularRateGain(Eigen::Vector3d::Zero(3));

    Eigen::Vector3d rate_Gain = controller_.getAngularRateGain();
    rate_Gain = rate_Gain.cwiseQuotient(px4AttitudeLimits);

    sep = "\n----------Equivalent PX4 Rate Gains-----------\n";
    std::cout << sep << rate_Gain.transpose() << sep;
  }

  /*
   if (controller_.isTrackEndEffector()) {
   double xE2 = std::pow(getPosXEndEffector(), 2);
   double cy = config.J_yy / config.mass;
   double cz = config.J_zz / config.mass;

   sep = "\n----Recommend Tracking EE Rotational Gains-------\n";
   std::cout << sep << "Kr_y:" << config.c_KP_r_yy + xE2 * config.c_KP_t_zz * cy << " Kr_z:"
   << config.c_KP_r_zz + xE2 * config.c_KP_t_yy * cz << " Kw_y:" << config.K_w_y + xE2 * config.K_v_z * cy
   << " Kw_z:" << config.K_w_z + xE2 * config.K_v_y * cz << sep;
   }
   */

  // Compute Natural Frequency and Damping Coefficients for Assisting Online Tuning
  sep = "\n----------Natural Frequencies-----------\n";
  std::cout << sep << controller_.computeNaturalFreq() << sep;

  sep = "\n----------Damping Coefficients-----------\n";
  std::cout << sep << controller_.computeDampingCoeff() << sep;

  ROS_INFO(" \n Gain changes Done in Impedance Controller!");
}

void ETankImpedanceControllerNode::publishControlSignals(Eigen::VectorXd& est_external_wrench,
                                                         Eigen::VectorXd& wrench_error,
                                                         Eigen::VectorXd& command_wrench) {
  spc_uav_comm::ControlSignals signals_msg;

  signals_msg.header.stamp = ros::Time::now();
  signals_msg.header.frame_id = "base_link";

  signals_msg.control_wrench.torque.x = command_wrench(0);
  signals_msg.control_wrench.torque.y = command_wrench(1);
  signals_msg.control_wrench.torque.z = command_wrench(2);
  signals_msg.control_wrench.force.x = command_wrench(3);
  signals_msg.control_wrench.force.y = command_wrench(4);
  signals_msg.control_wrench.force.z = command_wrench(5);

  signals_msg.wrench_track_error.torque.x = wrench_error(0);
  signals_msg.wrench_track_error.torque.y = wrench_error(1);
  signals_msg.wrench_track_error.torque.z = wrench_error(2);
  signals_msg.wrench_track_error.force.x = wrench_error(3);
  signals_msg.wrench_track_error.force.y = wrench_error(4);
  signals_msg.wrench_track_error.force.z = wrench_error(5);

  signals_msg.estimated_external_wrench.torque.x = est_external_wrench(0);
  signals_msg.estimated_external_wrench.torque.y = est_external_wrench(1);
  signals_msg.estimated_external_wrench.torque.z = est_external_wrench(2);
  signals_msg.estimated_external_wrench.force.x = est_external_wrench(3);
  signals_msg.estimated_external_wrench.force.y = est_external_wrench(4);
  signals_msg.estimated_external_wrench.force.z = est_external_wrench(5);

  signals_msg.tank1_energy = controller_.getTank1().getTankLevel();

  control_signals_pub.publish(signals_msg);
}

void ETankImpedanceControllerNode::sendControlWrenchTimerCallback(const ros::TimerEvent& event) {
  updateGUISetpoint();
  controller_.enableEndEffectorTrack(isTrackEndEffector());
  controller_.enableDisturbanceObserver(isEnableDistObserver());
  controller_.setEnableDistReject(isRejectDisturbances());
  sendControlWrench();
}

void ETankImpedanceControllerNode::publishControlSignalsTimerCallback(const ros::TimerEvent& event) {
  // Publish Control Signals
  Eigen::VectorXd estimated_wrench;
  estimated_wrench.resize(6);
  estimated_wrench = controller_.getEstimatedExternalWrench();

  Eigen::VectorXd wrench_error;
  wrench_error.resize(6);
  wrench_error = controller_.getWrenchTrackingError();

  publishControlSignals(estimated_wrench, wrench_error, _command_wrench);
}

void ETankImpedanceControllerNode::sendControlWrench() {

  if (isControllerActive())
    controller_.setControllerActive(true);
  else
    controller_.setControllerActive(false);
  // Compute elapsed time
  controller_.setDtI(computeElapsedTime());

  // Calculate control wrench
  Eigen::VectorXd command_wrench;
  controller_.calculateCommandWrench(&command_wrench, _H_B_E, _H_I_B, _T_B, getH_I_C(), getT_C(), getTdot_C());
  _command_wrench = command_wrench;
  // Either Use SITL interface OR RotorS interface
  publishSetPoints(command_wrench);

}
}

int main(int argc, char **argv) {

  // Initialize ROS Node
  ros::init(argc, argv, "etank_impedance_controller_node");
  etank_impedance_controller::ETankImpedanceControllerNode etank_impedance_controller_node;
  // Update Set Point + send at 100 Hz
  ros::Timer timer1 = etank_impedance_controller_node.nh_.createTimer(
      ros::Duration(0.01), &etank_impedance_controller::ETankImpedanceControllerNode::sendControlWrenchTimerCallback,
      &etank_impedance_controller_node);
  // Capture control signals at 20 Hz
  ros::Timer timer2 = etank_impedance_controller_node.nh_.createTimer(
      ros::Duration(0.05),
      &etank_impedance_controller::ETankImpedanceControllerNode::publishControlSignalsTimerCallback,
      &etank_impedance_controller_node);

  ros::spin();

  return 0;
}
