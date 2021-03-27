/*
 * etank_impedance_controller.cpp
 *
 *  Created on: August, 2018
 *      Author: ramy
 *
 *
 * The purpose of this class is to provide methods
 * that receive the set and desired points and produce the needed wrench.
 * This class should NOT know anything about px4, ROS, rotorS or Gazebo.
 * TODO: Put every disturbance observer in a separate function.
 */

#include "spc_uav_control/etank_impedance_controller.h"

namespace etank_impedance_controller {

GeomImpedanceController::GeomImpedanceController() {
  controller_active_ = false;

  flightMode = FREEFLIGHTMODE;

  _enable_end_effector_track = false;
  _distObsRunning = false;
  _enable_dist_obs = false;
  _enable_wrench_track = false;
  _enable_dist_reject = false;
  _enable_energy_tank = false;
  dt_i = 0.0;
  _G_r.setZero();
  _G_t.setZero();
  _G_c.setZero();

  _K_r.setZero();
  _K_t.setZero();
  _K0_obs.setZero();
  P_hat.setZero();
  W_hat_ext_B.setZero();

  _Kp_w.resize(6);
  _Kp_w.setZero();

  _Ki_w.resize(6);
  _Ki_w.setZero();

  W_cmd_B.setZero();

  W_err_int.setZero();
  W_err.setZero();
  W_ext_offset_B.setZero();
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
    P_hat.setZero();
  }
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
  //T_er = T_B;

  if (_enable_end_effector_track) {
    // The spring is connected between Psi_E and Psi_C
    H_er = H_er * H_B_E;
    H_er_inv = inverseH(H_er);
    //T_er = Adjoint(H_E_B) * T_er;
    // The damper is connected between Psi_B and Psi_C
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

  // Total Feedforward component of Control Wrench
  W_ff = -Adjoint(H_Bd_B).transpose() * W_g_I;

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
  // Linear Damper + Gravity Compensation
  W_B = W_ff + W_d;

  // Spatial Spring
  if (_enable_end_effector_track) {
    W_B += Adjoint(H_E_B).transpose() * W_p;
  } else {
    W_B += W_p;
  }

  // Estimated External Wrench
  computeEstimateExternalWrench(I_B * T_B);

  if (_enable_wrench_track) {
    // Compute Wrench Tracking Control Law
    Eigen::Matrix<double, 6, 1> W_tr = computeWrenchTrackingLaw();
    // Update Control Law
    W_B += W_tr;
  } else {
    // Reject Disturbances by Using the Kp,w Gains as switches/valves (-1.0<=kp_i,w <0.0)
    W_B += _Kp_w.cwiseProduct(W_ext_offset_B);
  }

  // Compute System Dynamics
  Eigen::Matrix<double, 6, 1> f__P__Wc_B;
  f__P__Wc_B = adjoint(T_B).transpose() * I_B * T_B + Adjoint(H_Bd_B).transpose() * W_g_I + W_B;

  // Update Integration
  updateDisturbanceObserver(f__P__Wc_B, I_B * T_B);

  // Update Energy Tank
  if (_enable_energy_tank) {
    Eigen::Matrix<double, 6, 1> T_hat = _K0_obs * P_hat;
    tank_1.updateTank(dt_i, T_B.transpose() * Kd * T_B, T_B, T_hat.transpose() * T_hat, T_hat);
  }
  // Return Control Wrench
  *command_wrench << W_B;
}

Eigen::Matrix<double, 6, 1> GeomImpedanceController::computeWrenchTrackingLaw() {

  Eigen::Matrix<double, 6, 1> W_tr;
  Eigen::Matrix<double, 6, 1> w1_bar;

  if (_enable_dist_obs & _enable_wrench_track) {
    W_err = W_hat_ext_B - W_cmd_B;
    w1_bar = _Kp_w.cwiseProduct(W_err) + _Ki_w.cwiseProduct(W_err_int);
    if (_Ki_w.norm() > 0.0) {
      W_err_int += dt_i * W_err;
    } else {
      W_err_int.setZero();
    }

  } else {
    w1_bar.setZero();
    W_err_int.setZero();
  }
  if (_enable_energy_tank) {
    W_tr = tank_1.connectTank(w1_bar, 1);
  } else {
    W_tr = w1_bar;
  }
  return W_tr;
}

void GeomImpedanceController::computeEstimateExternalWrench(Eigen::Matrix<double, 6, 1> P_B) {
  if (_enable_dist_obs) {
    W_hat_ext_B = _K0_obs * (P_B - P_hat);
    if (_enable_dist_reject) {
      W_ext_offset_B = W_hat_ext_B;
    }
  } else {
    W_hat_ext_B.setZero();
    P_hat = P_B;
    W_ext_offset_B.setZero();
  }
  // Debugging Disturbance Observer
  // std::cout <<W_hat_int_B.transpose()<< "\n";

}

void GeomImpedanceController::updateDisturbanceObserver(Eigen::Matrix<double, 6, 1> f__P__Wc_B,
                                                        Eigen::Matrix<double, 6, 1> P_B) {

  Eigen::Matrix<double, 6, 1> w2_bar;
  if (_enable_dist_obs) {
    w2_bar = f__P__Wc_B + _K0_obs * P_B;
    if (_enable_energy_tank) {
      w2_bar = tank_1.connectTank(w2_bar, 2);  // w2_bar is now w2
    }
    P_hat += dt_i * (-_K0_obs * P_hat + w2_bar);
  }
}

}

