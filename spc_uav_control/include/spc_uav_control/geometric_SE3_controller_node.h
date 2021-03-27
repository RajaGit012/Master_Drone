/*
 * geometric_SE3_controller_node.h
 *
 *  Created on: Jan 19, 2018
 *      Author: ramy
 *  Refactored on: May 2018
 *      Author: Asem
 */

#ifndef INCLUDE_SPC_UAV_CONTROL_GEOMETRIC_SE3_CONTROLLER_NODE_H_
#define INCLUDE_SPC_UAV_CONTROL_GEOMETRIC_SE3_CONTROLLER_NODE_H_

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <spc_uav_control/SE3_Control_GainsConfig.h>
#include <spc_uav_control/controller_node.h>
#include <spc_uav_control/geometric_SE3_controller.h>

namespace motion_controller{

class GeometricSE3ControllerNode : public ControllerNode {
 public:
  GeometricSE3ControllerNode();
  virtual ~GeometricSE3ControllerNode(){}


  void loadSE3ROSParameters();

  // Dynamic Reconfigure Callback Function
  void dynamicReconfigureSE3Callback(const spc_uav_control::SE3_Control_GainsConfig&, const uint32_t);

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
  dynamic_reconfigure::Server<spc_uav_control::SE3_Control_GainsConfig> motionDServer;


 private:
  // Instance of motion control system
  GeometricSE3Controller controller_;


};
}
#endif /* INCLUDE_SPC_UAV_CONTROL_GEOMETRIC_SE3_CONTROLLER_NODE_H_ */
