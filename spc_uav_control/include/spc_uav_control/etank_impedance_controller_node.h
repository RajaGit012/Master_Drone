/*
 * etank_impedance_controller_node.h
 *
 *  Created on: August, 2018
 *      Author: ramy
 */

#ifndef INCLUDE_SPC_UAV_CONTROL_ETANK_IMPEDANCE_CONTROLLER_NODE_H_
#define INCLUDE_SPC_UAV_CONTROL_ETANK_IMPEDANCE_CONTROLLER_NODE_H_

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <spc_uav_control/ET_Impedance_Control_GainsConfig.h>
#include <spc_uav_control/controller_node.h>
#include <spc_uav_control/etank_impedance_controller.h>
#include "spc_uav_comm/ControlSignals.h"

namespace etank_impedance_controller {

class ETankImpedanceControllerNode : public ControllerNode {
 public:
  /**
   * Constructor
   */
  ETankImpedanceControllerNode();
  /**
   * Destructor
   */
  virtual ~ETankImpedanceControllerNode() {
  }
  /**
   * Load from ROS parameter server
   */
  void loadImpedanceROSParameters();

  // Dynamic Reconfigure Callback Function
  void dynamicReconfigureImpedanceCallback(const spc_uav_control::ET_Impedance_Control_GainsConfig&, const uint32_t);

  // Compute the Control wrench
  void sendControlWrench();

  /**
   * Timer Callback: Update Setpoint from TF server and Send Control Wrench
   */
  void sendControlWrenchTimerCallback(const ros::TimerEvent&);
  /**
   * Timer Callback: Publish Control Signals for documentation
   */
  void publishControlSignalsTimerCallback(const ros::TimerEvent&);

  /**
   * get source of Command Frame
   * @return Desired Configuration
   */
  virtual Eigen::Matrix<double, 4, 4> getH_I_C() {
    return _H_I_D;
  }
  virtual Eigen::Matrix<double, 6, 1> getT_C() {
    return _T_D;
  }
  virtual Eigen::Matrix<double, 6, 1> getTdot_C() {
    return _Tdot_D;
  }

  /**
   * Construct the custom ETank control signals message and publish it
   * @param est_external_wrench: Estimated External Wrench
   * @param wrench_error: Error between Estimated and Desired Wrench
   * @param command_wrench: Commanded Control wrench
   */
  void publishControlSignals(Eigen::VectorXd& est_external_wrench,
      Eigen::VectorXd& wrench_error, Eigen::VectorXd& command_wrench);

 protected:
  ros::NodeHandle motion_nh_;
  // dynamic reconfigure server
  dynamic_reconfigure::Server<spc_uav_control::ET_Impedance_Control_GainsConfig> motionDServer;

 private:
  // Instance of motion control system
  GeomImpedanceController controller_;
  // Controller's Command Wrench
  Eigen::VectorXd _command_wrench;

};
}
#endif /* INCLUDE_SPC_UAV_CONTROL_ETANK_IMPEDANCE_CONTROLLER_NODE_H_ */
