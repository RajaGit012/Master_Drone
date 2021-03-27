/*
 * geometric_SO3_controller_node.h
 *
 *  Created on: Jun 26, 2018
 *      Author: ramy
 */

#ifndef INCLUDE_SPC_UAV_CONTROL_GEOMETRIC_SO3_CONTROLLER_NODE_H_
#define INCLUDE_SPC_UAV_CONTROL_GEOMETRIC_SO3_CONTROLLER_NODE_H_

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <spc_uav_control/SO3_Control_GainsConfig.h>
#include <spc_uav_control/controller_node.h>
#include <spc_uav_control/geometric_SO3_controller.h>

namespace motion_controller {

class GeometricSO3ControllerNode : public ControllerNode {

 public:
  GeometricSO3ControllerNode();
  virtual ~GeometricSO3ControllerNode() {
  }

  void loadSO3ROSParameters();

  // Dynamic Reconfigure Callback Function
  void dynamicReconfigureSO3Callback(const spc_uav_control::SO3_Control_GainsConfig&, const uint32_t);

  // Compute the Control wrench and publish the corresponding commands
  void sendControlWrench();

  virtual Eigen::Matrix<double, 4, 4> getH_I_C() {
    return _H_I_D;
  }

  virtual Eigen::Matrix<double, 6, 1> getT_C() {
    return _T_D;
  }

  virtual Eigen::Matrix<double, 6, 1> getTdot_C() {
    return _Tdot_D;
  }

 protected:
  ros::NodeHandle motion_nh_;
  // dynamic reconfigure server
  dynamic_reconfigure::Server<spc_uav_control::SO3_Control_GainsConfig> motionDServer;

 private:
  // Instance of motion control system
  GeometricSO3Controller controller_;

};

}

#endif /* INCLUDE_SPC_UAV_CONTROL_GEOMETRIC_SO3_CONTROLLER_NODE_H_ */
