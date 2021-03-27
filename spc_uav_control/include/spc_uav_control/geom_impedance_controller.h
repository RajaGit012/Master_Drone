/*
 *  geom_impedance_controller.h
 *
 *  Created on: June, 2018
 *      Author: ramy
 */

#ifndef INCLUDE_SPC_UAV_CONTROL_GEOM_IMPEDANCE_CONTROLLER_H_
#define INCLUDE_SPC_UAV_CONTROL_GEOM_IMPEDANCE_CONTROLLER_H_

#include <Eigen/Eigen>
#include <mav_msgs/eigen_mav_msgs.h>
#include "spc_uav_control/MathHelper.h"

// Define Constants
#define M_PI           3.14159265358979323846  /* pi */
#define FREEFLIGHTMODE 0.0
#define APPROACHMODE   1.0
#define VERTCIRCLEMODE    2.0

namespace impedance_controller {

class GeomImpedanceController {
 public:
  GeomImpedanceController();
  ~GeomImpedanceController();
  /**
   * !!! NOT USED IN SE3 CONTROLLER ONLY LEFT FOR REFERENCE
   * @param T: Thrust force in body-z-axis
   * @param B_z_d: Desired direction of commanded thrust force in body-z-axis
   */
  void computeTrajectoryTracking(double* T, Eigen::Vector3d* B_z_d) const;
  /**
   * !!! NOT USED IN SE3 CONTROLLER ONLY LEFT FOR REFERENCE
   * @param B_z_d: Desired direction of commanded thrust force in body-z-axis
   * @param tau:  Control torque vector
   */
  void computeAttitudeTracking(const Eigen::Vector3d& B_z_d, Eigen::Vector3d* tau) const;
  /**
   * CONTAINS MOTION CONTROLLER TO TRACK DESIRED CONFIGURATION
   * @param command_wrench: control thrust force and torques for underactuated UAVs
   */
  void calculateCommandWrench(Eigen::VectorXd* command_wrench, Eigen::Matrix<double, 4, 4> H_B_E,
                              Eigen::Matrix<double, 4, 4> H_I_B, Eigen::Matrix<double, 6, 1> T_B,
                              Eigen::Matrix<double, 4, 4> H_I_C, Eigen::Matrix<double, 6, 1> T_C,
                              Eigen::Matrix<double, 6, 1> Tdot_C);
  /***************************************************************************************************************************************/

  void setAngularRateGain(const Eigen::Vector3d& angularRateGain) {
    _kAngularRateGain = angularRateGain.cwiseQuotient(_kInertiaDiag);
  }

  /**
   *
   * @param inertiaDiag
   */
  void setInertiaDiag(const Eigen::Vector3d& inertiaDiag) {
    _kInertiaDiag = inertiaDiag;
  }
  /**
   *
   * @param mass
   */
  void setMass(double mass) {
    _kMass = mass;
  }
  /**
   *
   * @param velocityGain
   */
  void setVelocityGain(const Eigen::Vector3d& velocityGain) {
    _kVelocityGain = velocityGain / _kMass;
  }
  /**
   *
   * @return controller state
   */
  bool isControllerActive() const {
    return controller_active_;
  }
  /**
   * Used for debugging to check yaw angle
   * @return
   */
  double getYaw() const {
    return yaw_;
  }
  /**
   *
   * @param controllerActive
   */
  void setControllerActive(bool controllerActive) {
    controller_active_ = controllerActive;
  }

  void setGravity(double gravity) {
    _kGravity = gravity;
  }

  void setDtI(double dtI) {
    dt_i = dtI;
  }

  void setK_p_t(Eigen::Vector3d K_Pt) {
    _K_t = K_Pt/_kMass;
    _G_t = _K_t.asDiagonal();
    _G_t = 0.5 * _G_t.trace() * Eigen::Matrix3d::Identity() - _G_t;
  }

  void setK_p_r(Eigen::Vector3d K_Pr) {
    _K_r = K_Pr.cwiseQuotient(_kInertiaDiag);
    _G_r = _K_r.asDiagonal();
    _G_r = 0.5 * _G_r.trace() * Eigen::Matrix3d::Identity() - _G_r;
  }


  void setTrackEndEffector(bool trackEndEffector) {
    _trackEndEffector = trackEndEffector;
  }


  Eigen::Matrix<double, 1, 6> computeNaturalFreq(){
    Eigen::Matrix<double, 6, 1> nat_freq;
    nat_freq.setZero();
    nat_freq.head(3) = _K_r.cwiseSqrt();
    nat_freq.tail(3) = _K_t.cwiseSqrt();
    return nat_freq.transpose();
  }

  Eigen::Matrix<double, 1, 6> computeDampingCoeff(){
    Eigen::Matrix<double, 6, 1> damp_coeff;
    damp_coeff.setZero();
    damp_coeff.head(3) = 0.5*(_kAngularRateGain+_kOnboardAngularRateGain).cwiseQuotient(_K_r.cwiseSqrt());
    damp_coeff.tail(3) = 0.5*_kVelocityGain.cwiseQuotient(_K_t.cwiseSqrt());
    return damp_coeff.transpose();
  }

  const Eigen::Vector3d& getAngularRateGain() const {
    return _kAngularRateGain;
  }

  const Eigen::Vector3d& getOnboardAngularRateGain() const {
    return _kOnboardAngularRateGain;
  }

  void setOnboardAngularRateGain(const Eigen::Vector3d& onboardAngularRateGain) {
    _kOnboardAngularRateGain = onboardAngularRateGain;
  }

  bool isTrackEndEffector() const {
    return _trackEndEffector;
  }

 private:
  // controller's state
  bool controller_active_;
  bool _trackEndEffector;

  // NOT USED IN CURRENT VERSION
  double flightMode;
  // Gravitational constant
  double _kGravity;
  // UAV's mass [Kg]
  double _kMass;
  // Yaw Angle for debugging
  double yaw_;

  // Diagonal Elemnts of Inertia matrix
  Eigen::Vector3d _kInertiaDiag;
  // Controller position gains
  Eigen::Vector3d _kPositionGain;
  // Controller velocity gains
  Eigen::Vector3d _kVelocityGain;
  // Controller Angular Displacement Gains
  Eigen::Vector3d _kAttitudeGain;
  // Controller Angular velocity Gains
  Eigen::Vector3d _kAngularRateGain;

  // Controller Onboard Equivalent Angular velocity Gains
  Eigen::Vector3d _kOnboardAngularRateGain;


  Eigen::Vector3d _K_t;
  Eigen::Vector3d _K_r;

  Eigen::Matrix<double, 3, 3> _G_t;
  Eigen::Matrix<double, 3, 3> _G_r;
  Eigen::Matrix<double, 3, 3> _G_c;

  double dt_i;

};
}

#endif /* INCLUDE_SPC_UAV_CONTROL_GEOM_IMPEDANCE_CONTROLLER_H_ */
