/*
 * geom_impedance_controller_node.h
 *
 *  Created on: June, 2018
 *      Author: ramy
 */

#ifndef INCLUDE_SPC_UAV_CONTROL_GEOM_IMPEDANCE_CONTROLLER_NODE_H_
#define INCLUDE_SPC_UAV_CONTROL_GEOM_IMPEDANCE_CONTROLLER_NODE_H_

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <spc_uav_control/Impedance_Control_GainsConfig.h>
#include <spc_uav_control/controller_node.h>
#include <spc_uav_control/geom_impedance_controller.h>

namespace impedance_controller{

class GeomImpedanceControllerNode : public ControllerNode {
 public:
  GeomImpedanceControllerNode();
  virtual ~GeomImpedanceControllerNode(){}


  void loadImpedanceROSParameters();

  // Dynamic Reconfigure Callback Function
  void dynamicReconfigureImpedanceCallback(const spc_uav_control::Impedance_Control_GainsConfig&, const uint32_t);

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
  dynamic_reconfigure::Server<spc_uav_control::Impedance_Control_GainsConfig> motionDServer;


 private:
  // Instance of motion control system
  GeomImpedanceController controller_;


};
}
#endif /* INCLUDE_SPC_UAV_CONTROL_GEOM_IMPEDANCE_CONTROLLER_NODE_H_ */
