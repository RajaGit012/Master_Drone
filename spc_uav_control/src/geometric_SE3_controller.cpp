/*
 * geometric_SE3_controller.cpp
 *
 *  Created on: Jan 19, 2018
 *      Author: ramy
 *  Refactored on: May 2018
 *      Author: Asem
 *
 * The purpose of this class is to provide methods
 * that receive the set and desired points and produce the needed wrench.
 * This class should NOT know anything about px4, ROS, rotorS or Gazebo.
 * TODO: Put every disturbance observer in a separate function.
 */

#include "spc_uav_control/geometric_SE3_controller.h"

namespace motion_controller {

GeometricSE3Controller::GeometricSE3Controller() {
  controller_active_ = false;

  flightMode = FREEFLIGHTMODE;
  _kGravity = 9.81;

  _trackEndEffector = false;

  deltaW.setZero();  // reset integration
  etaW.setZero();
  _DistObsRunning = false;
  dt_i = 0.0;
  _kDistObsGain_0.setZero();
  _kDistObsGain_1.setZero();
  disturbance_observer_type = NODISTURBANCEOBSERVER;
  DO_ON = 0.0;
}

GeometricSE3Controller::~GeometricSE3Controller() {

}

void GeometricSE3Controller::calculateCommandWrench(
    Eigen::VectorXd* command_wrench, Eigen::Matrix<double, 4, 4> H_B_E,
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
  // Precompensation Wrench of Gravity
  Eigen::Matrix<double, 6, 1> W_dist_hat;

  if (!controller_active_) {
    deltaW.setZero();  // reset integration
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
      T_er = Adjoint(H_E_B) * T_er;
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
    W_ff_3 =
        I_B
            * (Adjoint(H_er_inv) * Tdot_C
                - adjoint(T_er) * Adjoint(H_er_inv) * T_C);
    // Total Feedforward component of Control Wrench
    W_ff = W_ff_1 + W_ff_2;

    // Proportional component of Control Wrench
    Eigen::Matrix3d R_er = H_er.topLeftCorner(3, 3);
    Eigen::Vector3d xi_er = H_er.topRightCorner(3, 1);
    W_p.head(3) = -unskew(antisym(_kAttitudeGain.asDiagonal() * R_er));
    W_p.tail(3) = -1.0 * _kPositionGain.asDiagonal() * R_er.transpose() * xi_er;

    // Disturbance Observer

    if (DO_ON == 1.0) {

      if (!_DistObsRunning) {
        _DistObsRunning = true;

        switch (disturbance_observer_type) {
          case DISTURBANCEOBSERVER_1:
          case DISTURBANCEOBSERVER_2:

            deltaW = I_B * T_B;
            etaW.setZero();
            W_dist_hat.setZero();

            break;
          case DISTURBANCEOBSERVER_3:
            W_dist_hat.setZero();
            deltaW.setZero();
            etaW = I_B * T_B;
            break;

        }

      } else {

        switch (disturbance_observer_type) {
          case DISTURBANCEOBSERVER_1:
          case DISTURBANCEOBSERVER_2:

            W_dist_hat = _kDistObsGain_0 * (I_B * T_B - deltaW)
                + _kDistObsGain_1 * etaW;

            break;
          case DISTURBANCEOBSERVER_3:

            W_dist_hat = deltaW;

            break;

        }

      }
    } else {
      _DistObsRunning = false;
      W_dist_hat.setZero();

      switch (disturbance_observer_type) {
        case DISTURBANCEOBSERVER_1:
        case DISTURBANCEOBSERVER_2:

          deltaW = I_B * T_B;
          etaW.setZero();

          break;
        case DISTURBANCEOBSERVER_3:

          deltaW.setZero();
          etaW = I_B * T_B;
          break;

      }

    }

    // Control Wrench in Body Frame
    W_B = W_ff - W_dist_hat;

    if (_trackEndEffector) {
      W_B += Adjoint(H_E_B).transpose() * (W_p + W_d + W_ff_3);
    } else {
      W_B += W_p + W_d + W_ff_3;
    }

    // Update DIsturbance Observer
    switch (disturbance_observer_type) {
      case DISTURBANCEOBSERVER_1:
        if (DO_ON == 1.0) {

          Eigen::Matrix<double, 6, 1> deltaW_dot = W_p + W_d + W_ff_3;

          if (_trackEndEffector) {
            deltaW_dot = Adjoint(H_E_B).transpose() * deltaW_dot;
          }

          deltaW = deltaW + dt_i * deltaW_dot;
          etaW.setZero();

        }
        break;

      case DISTURBANCEOBSERVER_2:
        if (DO_ON == 1.0) {

          Eigen::Matrix<double, 6, 1> deltaW_dot = W_p + W_d + W_ff_3;
          if (_trackEndEffector) {
            deltaW_dot = Adjoint(H_E_B).transpose() * deltaW_dot;
          }

          Eigen::Matrix<double, 6, 1> etaW_dot = I_B * T_B - deltaW;

          deltaW = deltaW + dt_i * deltaW_dot;
          etaW = etaW + dt_i * etaW_dot;
        }
        break;

      case DISTURBANCEOBSERVER_3:
        if (DO_ON == 1.0) {
          Eigen::Matrix<double, 6, 1> nu_bar1 = (etaW - I_B * T_B).cwiseAbs();
          Eigen::Matrix<double, 6, 1> nu_bar2 = signArray(etaW - I_B * T_B);
          Eigen::Matrix<double, 6, 1> nu_i = -_kDistObsGain_0
              * nu_bar2.asDiagonal() * nu_bar1.cwiseSqrt() + deltaW;

          Eigen::Matrix<double, 6, 1> etaW_dot = -W_dist_hat + nu_i;

          if (_trackEndEffector) {
            etaW_dot += Adjoint(H_E_B).transpose() * (W_p + W_d + W_ff_3);
          } else {
            etaW_dot += W_p + W_d + W_ff_3;
          }

          Eigen::Matrix<double, 6, 1> deltaW_dot = -_kDistObsGain_1
              * signArray(deltaW - nu_i);

          etaW = etaW + dt_i * etaW_dot;
          deltaW = deltaW + dt_i * deltaW_dot;

        }
        break;

    }

    // Return Control Wrench
    *command_wrench << W_B;
  }
}
}

