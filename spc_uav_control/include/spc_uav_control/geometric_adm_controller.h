/*
 * geometric_adm_controller.h
 *
 *  Created on: Jan 27, 2018
 *      Author: ramy
 *  Refactored on: May 2018
 *      Author: Asem
 */

#ifndef INCLUDE_SPC_UAV_CONTROL_GEOMETRIC_ADM_CONTROLLER_H_
#define INCLUDE_SPC_UAV_CONTROL_GEOMETRIC_ADM_CONTROLLER_H_

#include <Eigen/Eigen>
#include <mav_msgs/eigen_mav_msgs.h>
#include <iostream>
#include "spc_uav_control/MathHelper.h"

class GeometricAdmittanceController {
 public:
  GeometricAdmittanceController();
  ~GeometricAdmittanceController(){}

  void impedanceFilter(Eigen::Matrix<double, 4, 4> H_I_D,
                       Eigen::Matrix<double, 6, 1> T_D,
                       Eigen::Matrix<double, 6, 1> Tdot_D,
                       Eigen::Matrix<double, 6, 1> W_D);

  void setInirtialTensor(double mass, Eigen::Vector3d inirtiaDiag);

  void setK_D(Eigen::Vector3d K_Dr, Eigen::Vector3d K_Dt);

  void setK_G_t(Eigen::Vector3d K_Pt);

  void setK_G_r(Eigen::Vector3d K_Pr);

  void setDt(double dt);

  void setControllerActive(bool d);

  Eigen::Matrix<double, 4, 4> getH_I_C(){
    return _H_I_C;
  }
  Eigen::Matrix<double, 6, 1> getT_C(){
    return _T_C;
  }
  Eigen::Matrix<double, 6, 1> getTdot_C(){
    return _Tdot_C;
  }


 private:
  Eigen::Matrix<double, 4, 4> _H_I_C;
  Eigen::Matrix<double, 6, 1> _T_C;
  Eigen::Matrix<double, 6, 1> _Tdot_C;

  Eigen::Matrix<double, 4, 4> _H_C_D;
  Eigen::Matrix<double, 6, 1> _T_D_C_D;

  Eigen::Matrix<double, 6, 6> _K_D;
  Eigen::Matrix<double, 3, 3> _G_t;
  Eigen::Matrix<double, 3, 3> _G_r;
  Eigen::Matrix<double, 3, 3> _G_c;
  Eigen::Matrix<double, 6, 6> _I_C;

  double _dt;
  bool _filter_active;
};

#endif /* INCLUDE_SPC_UAV_CONTROL_GEOMETRIC_ADM_CONTROLLER_H_ */
