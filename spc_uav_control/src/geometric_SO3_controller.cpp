/*
 * geometric_SO3_controller.cpp
 *
 *  Created on: Jun 26, 2018
 *      Author: ramy
 */

#include "spc_uav_control/geometric_SO3_controller.h"

namespace motion_controller {

GeometricSO3Controller::GeometricSO3Controller() {
  controller_active_ = false;

  flightMode = FREEFLIGHTMODE;
  _kGravity = 9.81;

  dt_i = 0.0;
}

GeometricSO3Controller::~GeometricSO3Controller() {

}

void GeometricSO3Controller::calculateCommandWrench(Eigen::VectorXd* command_wrench, Eigen::Matrix<double, 4, 4> H_I_B,
                                                    Eigen::Matrix<double, 6, 1> T_B, Eigen::Matrix<double, 4, 4> H_I_C,
                                                    Eigen::Matrix<double, 6, 1> T_C,
                                                    Eigen::Matrix<double, 6, 1> Tdot_C) {
  assert(command_wrench);
  command_wrench->resize(6);
  if (!controller_active_) {
  }
  // Check Flight Mode from Simulation GUI
  if ((flightMode == VERTCIRCLEMODE) || (flightMode == APPROACHMODE)) {
    // NOT USED IN CURRENT VERSION
  } else  // Free flight
  {
    // Define the control thrust and desired third body-fixed-axis
    double T;  // [N]
    Eigen::Vector3d B_z_d;  // Unit Vector

    // Compute the outer loop variables
    computeTrajectoryTracking(&T, &B_z_d, H_I_B, T_B, H_I_C, T_C, Tdot_C);

    // Attitude tracking.
    Eigen::Vector3d tau;  // [N.m]
    computeAttitudeTracking(B_z_d, &tau, H_I_B, T_B, H_I_C, T_C, Tdot_C);

    *command_wrench << tau,0.0,0.0,T;
  }

}

void GeometricSO3Controller::computeTrajectoryTracking(double* T, Eigen::Vector3d* B_z_d,
                                                       Eigen::Matrix<double, 4, 4> H_I_B,
                                                       Eigen::Matrix<double, 6, 1> T_B,
                                                       Eigen::Matrix<double, 4, 4> H_I_C,
                                                       Eigen::Matrix<double, 6, 1> T_C,
                                                       Eigen::Matrix<double, 6, 1> Tdot_C){
  assert(T);
  assert(B_z_d);

  // Extract UAV Rotation matrix
  const Eigen::Matrix3d R_I_B = H_I_B.topLeftCorner(3, 3);
  const Eigen::Matrix3d R_I_C = H_I_C.topLeftCorner(3, 3);

  // Transform linear velocity from body frame to world frame.
  const Eigen::Vector3d I_v = R_I_B * T_B.tail(3);

  // Compute translational tracking errors in World Frame
  const Eigen::Vector3d e_p = H_I_B.topRightCorner(3, 1) - H_I_C.topRightCorner(3, 1);
  const Eigen::Vector3d e_v = I_v - R_I_C*T_C.tail(3);

  // Define Feedforward Acceleration
  Eigen::Vector3d I_a_ref = R_I_C*Tdot_C.tail(3);

  // Compute the the desired acceleration in World Frame
  const Eigen::Vector3d I_a_d = -_kPositionGain.cwiseProduct(e_p) - _kVelocityGain.cwiseProduct(e_v)
      + _kGravity * Eigen::Vector3d::UnitZ() + I_a_ref;

  // Compute the control thrust T by projecting the acceleration onto the body z-axis.
  *T = _kMass * I_a_d.dot(R_I_B.col(2));

  // Compute the desired thrust direction B_z_d from the desired acceleration I_a_d.
  *B_z_d = I_a_d;
  B_z_d->normalize();
}

void GeometricSO3Controller::computeAttitudeTracking(const Eigen::Vector3d& B_z_d, Eigen::Vector3d* tau,Eigen::Matrix<double, 4, 4> H_I_B,
                             Eigen::Matrix<double, 6, 1> T_B,
                             Eigen::Matrix<double, 4, 4> H_I_C,
                             Eigen::Matrix<double, 6, 1> T_C,
                             Eigen::Matrix<double, 6, 1> Tdot_C)const{

  assert(tau);

  using namespace MathHelper;

  // Extract UAV Rotation matrix
  const Eigen::Matrix3d R_I_B = H_I_B.topLeftCorner(3, 3);
  //Compute the the desired heading direction B_x_d from the commanded yaw.

  //Eigen::Vector3f yaw_pitch_roll =H_I_C.eulerAngles(2,1,0);
  const double yaw_ref = 0.0;//yaw_pitch_roll(0);
  const Eigen::Vector3d B_x_d(std::cos(yaw_ref), std::sin(yaw_ref), 0.0);


  //Compute B_y_d which is perpendicular to B_z_d and B_x_d.
    Eigen::Vector3d B_y_d = B_z_d.cross(B_x_d);
    B_y_d.normalize();

    // Compute the desired attitude R_I_D.
    Eigen::Matrix3d R_I_D;
    R_I_D.col(0) = B_y_d.cross(B_z_d);
    R_I_D.col(1) = B_y_d;
    R_I_D.col(2) = B_z_d;

    // Compute the attitude tracking errors e_R and e_omega.
    Eigen::Matrix3d e_R_matrix = 0.5 * (R_I_D.transpose() * R_I_B - R_I_B.transpose() * R_I_D);
    Eigen::Vector3d e_R = unskew(e_R_matrix);

    // Compute the angular velocity tracking error
    const Eigen::Vector3d omega_ref = T_C(2) * Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d omega = T_B.head(3);
    const Eigen::Vector3d e_omega = omega - R_I_B.transpose() * R_I_D * omega_ref;

    // Compute the command torque.
    *tau = -_kAttitudeGain.cwiseProduct(e_R) - _kAngularRateGain.cwiseProduct(e_omega)
        + omega.cross(_kInertiaDiag.asDiagonal() * omega);

}

}

