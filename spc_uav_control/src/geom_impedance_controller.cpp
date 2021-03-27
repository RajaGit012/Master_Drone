/*
 * geom_impedance_controller.cpp
 *
 *  Created on: June, 2018
 *      Author: ramy
 *
 *
 * The purpose of this class is to provide methods
 * that receive the set and desired points and produce the needed wrench.
 * This class should NOT know anything about px4, ROS, rotorS or Gazebo.
 * TODO: Put every disturbance observer in a separate function.
 */

#include "spc_uav_control/geom_impedance_controller.h"

namespace impedance_controller {

GeomImpedanceController::GeomImpedanceController() {
  controller_active_ = false;

  flightMode = FREEFLIGHTMODE;
  _kGravity = 9.81;

  _trackEndEffector = true;
  dt_i = 0.0;
  _G_r.setZero();
  _G_t.setZero();
  _G_c.setZero();

  _K_r.setZero();
  _K_t.setZero();
}

GeomImpedanceController::~GeomImpedanceController() {

}

void GeomImpedanceController::calculateCommandWrench(Eigen::VectorXd* command_wrench, Eigen::Matrix<double, 4, 4> H_B_E,
                                                     Eigen::Matrix<double, 4, 4> H_I_B, Eigen::Matrix<double, 6, 1> T_B,
                                                     Eigen::Matrix<double, 4, 4> H_I_C, Eigen::Matrix<double, 6, 1> T_C,
                                                     Eigen::Matrix<double, 6, 1> Tdot_C) {
  assert(command_wrench);
  command_wrench->resize(6);

  // Control Wrench in Body Frame
  Eigen::Matrix<double, 6, 1> W_B;
  // Proportional component of Control Wrench
  Eigen::Matrix<double, 6, 1> W_p;
  // Derivative component of Control Wrench
  Eigen::Matrix<double, 6, 1> W_d;
  // Feedforward component of Control Wrench
  Eigen::Matrix<double, 6, 1> W_ff;
  // Precompensation Wrench of Gravity
  Eigen::Matrix<double, 6, 1> W_g_I;

  if (!controller_active_) {
    // reset integration
  }
  // Check Flight Mode from Simulation GUI
  if ((flightMode == VERTCIRCLEMODE) || (flightMode == APPROACHMODE)) {
    // NOT USED IN CURRENT VERSION
  } else  // Free flight
  {
    using namespace MathHelper;
    Eigen::Matrix<double, 4, 4> H_E_B = inverseH(H_B_E);

    // Define Generalized Inertia Tensor
    Eigen::Matrix<double, 6, 6> I_B;
    I_B.setZero();
    I_B.topLeftCorner(3, 3) = _kInertiaDiag.asDiagonal();
    I_B.bottomRightCorner(3, 3) = _kMass * Eigen::Matrix3d::Identity();

    // Define Right Configuration Error and Velocity Error
    Eigen::Matrix4d H_er;
    Eigen::Matrix4d H_er_inv;
    Eigen::Matrix<double, 6, 1> T_er;

    H_er = inverseH(H_I_C) * H_I_B;
    H_er_inv = inverseH(H_er);
    T_er = T_B - Adjoint(H_er_inv) * T_C;

    if (_trackEndEffector) {
      H_er = H_er * H_B_E;
      H_er_inv = inverseH(H_er);
      //T_er = Adjoint(H_E_B) * T_er;
    }

    // Control Gain matrix (for damping)
    Eigen::Matrix<double, 6, 6> Kd;
    Kd.setZero();
    Kd.topLeftCorner(3, 3) = _kAngularRateGain.asDiagonal();
    Kd.bottomRightCorner(3, 3) = _kVelocityGain.asDiagonal();

    // Compute derivative component of Control Wrench
    W_d = -Kd * T_er;

    // Precompensation Wrench of Gravity
    Eigen::Matrix4d H_Bd_B;
    H_Bd_B.setIdentity();
    H_Bd_B.topLeftCorner(3, 3) = H_I_B.topLeftCorner(3, 3);
    W_g_I << 0, 0, 0, 0, 0, -_kMass * _kGravity;

    // Feedforward components of Control Wrench
    Eigen::Matrix<double, 6, 1> W_ff_1;
    Eigen::Matrix<double, 6, 1> W_ff_2;
    Eigen::Matrix<double, 6, 1> W_ff_3;

    W_ff_1 = -adjoint(T_B).transpose() * I_B * T_B;
    W_ff_2 = -Adjoint(H_Bd_B).transpose() * W_g_I;
    W_ff_3.setZero();
    //W_ff_3 = I_B * (Adjoint(H_er_inv) * Tdot_C - adjoint(T_er) * Adjoint(H_er_inv) * T_C);
    // Total Feedforward component of Control Wrench
    W_ff = W_ff_2;

    // Proportional component of Control Wrench
    Eigen::Matrix3d R_er = H_er.topLeftCorner(3, 3);
    Eigen::Vector3d xi_er = H_er.topRightCorner(3, 1);
    Eigen::Matrix<double, 3, 3> m_tild;
    Eigen::Matrix<double, 3, 3> f_tild;

    // Spring Wrench Calculation
    m_tild = -(2.0 * antisym(_G_r * R_er) + antisym(_G_t * R_er.transpose() * skew(xi_er) * skew(xi_er) * R_er)
        + 2.0 * antisym(_G_c * skew(xi_er) * R_er));
    f_tild = -(R_er.transpose() * antisym(_G_t * skew(xi_er)) * R_er
        + antisym(_G_t * R_er.transpose() * skew(xi_er) * R_er) + 2.0 * antisym(_G_c * R_er));
    W_p << unskew(m_tild), unskew(f_tild);

    // Control Wrench in Body Frame
    W_B = W_ff + W_d;

    if (_trackEndEffector) {
      W_B += Adjoint(H_E_B).transpose() * (W_p + W_ff_3);
    } else {
      W_B += W_p + W_ff_3;
    }


    // Debugging whether command set points reach px4 correctly or not
   // std::string sep = "\n----------------------------------------\n";
    //std::cout <<sep<< T_B(0)<<", "<< T_B(1)<<", "<< T_B(2) << sep;

    // Return Control Wrench
    *command_wrench << W_B;
  }
}

}

