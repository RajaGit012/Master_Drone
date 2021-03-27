/*
 * geometric_adm_controller.cpp
 *
 *  Created on: Jan 27, 2018
 *      Author: ramy
 *  Refactored on: May 2018
 *      Author: Asem
 */
#include "spc_uav_control/geometric_adm_controller.h"

GeometricAdmittanceController::GeometricAdmittanceController() {
  _filter_active = false;

  _dt = 0;
  _H_I_C.setIdentity();
  _T_C.setZero();
  _Tdot_C.setZero();
  _I_C.setIdentity();
  _K_D.setZero();

  _H_C_D.setIdentity();
  _T_D_C_D.setZero();

  //fix K_Pc as zeros:
  Eigen::Matrix<double, 3, 3> K_Pc;
  K_Pc.setZero();
  _G_c = 0.5*K_Pc.trace()*Eigen::Matrix3d::Identity() - K_Pc;
}

void GeometricAdmittanceController::setInirtialTensor(double mass, Eigen::Vector3d inirtiaDiag){
  _I_C.topLeftCorner(3, 3) = inirtiaDiag.asDiagonal();
  _I_C.bottomRightCorner(3, 3) = mass * Eigen::Matrix3d::Identity();
}

void GeometricAdmittanceController::setK_D(Eigen::Vector3d K_Dr, Eigen::Vector3d K_Dt){
  _K_D.topLeftCorner(3, 3) = K_Dr.asDiagonal();
  _K_D.bottomRightCorner(3, 3) = K_Dt.asDiagonal();
}

void GeometricAdmittanceController::setK_G_t(Eigen::Vector3d K_Pt){
  _G_t = K_Pt.asDiagonal();
  _G_t = 0.5*_G_t.trace()*Eigen::Matrix3d::Identity() - _G_t;
}

void GeometricAdmittanceController::setK_G_r(Eigen::Vector3d K_Pr){
  _G_r = K_Pr.asDiagonal();
  _G_r = 0.5*_G_r.trace()*Eigen::Matrix3d::Identity() - _G_r;
}

void GeometricAdmittanceController::setDt(double dt){
  _dt = dt;
}

void GeometricAdmittanceController::setControllerActive(bool d){
  _filter_active = d;
}

/*
 * Impedance filter (By Asem Khattab)
 * Frames: S: wrench sensor, B: Body, D: Desired, C: Compliance (or Command).
 * The function is called whenever a new value is received from the wrench sensor.
 */
void GeometricAdmittanceController::impedanceFilter(Eigen::Matrix<double, 4, 4> H_I_D,
                                                    Eigen::Matrix<double, 6, 1> T_D,
                                                    Eigen::Matrix<double, 6, 1> Tdot_D,
                                                    Eigen::Matrix<double, 6, 1> W_D) {
//  _H_I_C = _H_I_D;
//  _T_C = _T_D;
//  _Tdot_C = _Tdot_D;
  if (!_filter_active){
    _H_C_D.setIdentity();
    _T_D_C_D.setZero();
    _H_I_C = H_I_D;
    _T_C = T_D;
    _Tdot_C = Tdot_D;
    return;
  }


  using namespace MathHelper;
  Eigen::Matrix<double, 3, 3> R_C_D;
  Eigen::Vector3d P_C_D;
  Eigen::Matrix<double, 3, 3> m_tild;
  Eigen::Matrix<double, 3, 3> f_tild;

  Eigen::Matrix<double, 6, 1> W_spring;
  Eigen::Matrix<double, 6, 1> W_damper;
  Eigen::Matrix<double, 6, 1> Tdot_D_C_D;
  Eigen::Vector3d omega;
  Eigen::Matrix<double, 4, 4> Hdot_C_D;
  Eigen::Vector3d pos;
  Eigen::Vector3d OMEGA;
  Eigen::Matrix3d rot;

  // Transform to the wanted frames
  _H_I_C = H_I_D*inverseH(_H_C_D);
  _T_C = Adjoint(_H_C_D)*(T_D-_T_D_C_D);
  _Tdot_C = Adjoint(_H_C_D)*(Tdot_D-Tdot_D_C_D)+Adjoint(_H_C_D)*adjoint(_T_D_C_D)*(T_D-_T_D_C_D);
//  // only pass the translational part;
//  _H_I_C.topLeftCorner(3, 3) = _H_I_D.topLeftCorner(3, 3);
//  _T_C.topLeftCorner(3, 1) = _T_D.topLeftCorner(3, 1);
//  _Tdot_C.topLeftCorner(3, 1) = _Tdot_D.topLeftCorner(3, 1);

  R_C_D = _H_C_D.topLeftCorner(3, 3);
  P_C_D = _H_C_D.topRightCorner(3, 1);

  //std::cout << "------H_C_D-----\n";
  //std::cout << _H_C_D << '\n';

  // Spring Wrench Calculation
  m_tild = (2.0 * antisym(_G_r * R_C_D)
      + antisym(_G_t * R_C_D.transpose() * skew(P_C_D) * skew(P_C_D) * R_C_D)
      + 2.0 * antisym(_G_c * skew(P_C_D) * R_C_D));
  f_tild = (R_C_D.transpose() * antisym(_G_t * skew(P_C_D)) * R_C_D
      + antisym(_G_t * R_C_D.transpose() * skew(P_C_D) * R_C_D)
      + 2.0 * antisym(_G_c * R_C_D));
  W_spring << unskew(m_tild), unskew(f_tild);

  //std::cout << "------W_spring-----\n";
  //std::cout << W_spring << '\n';


  // Damper Wrench Calculation
  W_damper = _K_D*_T_D_C_D;

  //std::cout << "------W_damper-----\n";
  //std::cout << W_damper << '\n';

  // calculating and integrating Tdot_D_C_D
  //TODO: Precalculate this _I_C.inverse() (very easy);
  Tdot_D_C_D = _I_C.inverse()*(W_D - W_spring - W_damper);
  omega = _T_D_C_D.topLeftCorner(3, 1);          // omega[i-1]


  // calculating and integrating Hdot_C_D
  Hdot_C_D = _H_C_D*skewT(_T_D_C_D); // Hdot_D_C[i]
  pos = P_C_D + _dt*Hdot_C_D.topRightCorner(3, 1);
  OMEGA = (_dt/(double)2.0)*(_T_D_C_D.topLeftCorner(3, 1) + omega) + (_dt*_dt/(double)12.0)*(skew(_T_D_C_D.topLeftCorner(3, 1))*omega);
  rot = expMap(OMEGA)*R_C_D;
  _H_C_D.topLeftCorner(3, 3) = rot; //Eigen::Matrix3d::Identity();
  _H_C_D.topRightCorner(3, 1) = pos;

  //std::cout << "------_H_C_D after integration-----\n";
  //std::cout << _H_C_D << '\n';
  _T_D_C_D = _T_D_C_D + _dt*Tdot_D_C_D;                          // T_D_C_D[i]

  //std::cout << "------_T_D_C_D-----\n";
  //std::cout << _T_D_C_D << '\n';

}




