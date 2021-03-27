/*
 * geometric_adm_controller_node.h
 *
 *  Created on: Mar 8, 2018
 *      Author: ramy
 *  Refactored on: May 2018
 *      Author: Asem
 */

#ifndef INCLUDE_SPC_UAV_CONTROL_GEOMETRIC_ADM_CONTROLLER_NODE_H_
#define INCLUDE_SPC_UAV_CONTROL_GEOMETRIC_ADM_CONTROLLER_NODE_H_


#include <spc_uav_control/Admittance_Control_GainsConfig.h>
#include <spc_uav_control/geometric_SE3_controller_node.h>
#include <spc_uav_control/geometric_adm_controller.h>


class GeometricAdmittanceControllerNode : public motion_controller::GeometricSE3ControllerNode {
 public:

  GeometricAdmittanceControllerNode();
  ~GeometricAdmittanceControllerNode(){}

  // Dynamic Reconfigure Callback Function
  void dynamicReconfigureAdmCallback(spc_uav_control::Admittance_Control_GainsConfig&, uint32_t);

  // External Wrench Callback FUnction
  void externalWrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr&);

  // publish command frame
  void broadcastCommandTrajectory();

  virtual Eigen::Matrix<double, 4, 4> getH_I_C() {
    return interaction_controller_.getH_I_C();
  }

  virtual Eigen::Matrix<double, 6, 1> getT_C() {
    return interaction_controller_.getT_C();
  }

  virtual Eigen::Matrix<double, 6, 1> getTdot_C() {
    return interaction_controller_.getTdot_C();
  }


 protected:
  ros::NodeHandle adm_nh_;
  dynamic_reconfigure::Server<spc_uav_control::Admittance_Control_GainsConfig> admDServer;

 private:
  GeometricAdmittanceController interaction_controller_;
  ros::Subscriber external_wrench_sub;
  ros::Time adm_last_time;

};


#endif /* INCLUDE_SPC_UAV_CONTROL_GEOMETRIC_ADM_CONTROLLER_NODE_H_ */
