/*
 *  etank_impedance_controller.h
 *
 *  Created on: August, 2018
 *      Author: ramy
 */

#ifndef INCLUDE_SPC_UAV_CONTROL_ETANK_IMPEDANCE_CONTROLLER_H_
#define INCLUDE_SPC_UAV_CONTROL_ETANK_IMPEDANCE_CONTROLLER_H_

#include <Eigen/Eigen>
#include <mav_msgs/eigen_mav_msgs.h>
#include "spc_uav_control/MathHelper.h"
#include "spc_uav_control/energy_tank.h"

// Define Constants
#define M_PI           3.14159265358979323846  /* pi */
#define FREEFLIGHTMODE 0.0
#define APPROACHMODE   1.0
#define VERTCIRCLEMODE    2.0

namespace etank_impedance_controller {

class GeomImpedanceController {
 public:
  GeomImpedanceController();
  ~GeomImpedanceController();
  /**
   * Contains full observer-based wrench/impedance controller
   * @param command_wrench: control thrust force and torques for underactuated UAVs
   */
  void calculateCommandWrench(Eigen::VectorXd* command_wrench, Eigen::Matrix<double, 4, 4> H_B_E,
                              Eigen::Matrix<double, 4, 4> H_I_B, Eigen::Matrix<double, 6, 1> T_B,
                              Eigen::Matrix<double, 4, 4> H_I_C, Eigen::Matrix<double, 6, 1> T_C,
                              Eigen::Matrix<double, 6, 1> Tdot_C);
  /**
   * Computes Tracking Control Law
   * @return Wrench Regulating control Law
   */
  Eigen::Matrix<double, 6, 1> computeWrenchTrackingLaw();

  /**
   * Computes Estimated Wrench only
   * @param P_B: Actual Body Generalized Momentum
   */
  void computeEstimateExternalWrench(Eigen::Matrix<double, 6, 1> P_B);
  /**
   * Updates the state/dynamics of the disturbance observer
   * @param f__P__Wc_B: Function of Actual momentum and Control law
   * @param P_B: Actual Body Generalized Momentum
   */
  void updateDisturbanceObserver(Eigen::Matrix<double, 6, 1> f__P__Wc_B, Eigen::Matrix<double, 6, 1> P_B);
  /**
   * Compute Natural Frequency
   * @return
   */
  Eigen::Matrix<double, 1, 6> computeNaturalFreq() {
    Eigen::Matrix<double, 6, 1> nat_freq;
    nat_freq.setZero();
    nat_freq.head(3) = _K_r.cwiseSqrt();
    nat_freq.tail(3) = _K_t.cwiseSqrt();
    return nat_freq.transpose();
  }

  /**
   * Compute Damping Coefficients
   * @return
   */
  Eigen::Matrix<double, 1, 6> computeDampingCoeff() {
    Eigen::Matrix<double, 6, 1> damp_coeff;
    damp_coeff.setZero();
    damp_coeff.head(3) = 0.5 * (_kAngularRateGain + _kOnboardAngularRateGain).cwiseQuotient(_K_r.cwiseSqrt());
    damp_coeff.tail(3) = 0.5 * _kVelocityGain.cwiseQuotient(_K_t.cwiseSqrt());
    return damp_coeff.transpose();
  }

  /***************************************************************************************************************************************/
  /***************************************************** Setters ***************************************************************/

  /**
   * Normalizes gain by inertia
   * @param angularRateGain
   */
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
   * Normalizes Velocity gains by mass
   * @param velocityGain
   */
  void setVelocityGain(const Eigen::Vector3d& velocityGain) {
    _kVelocityGain = velocityGain / _kMass;
  }
  /**
   *
   * @param controllerActive
   */
  void setControllerActive(bool controllerActive) {
    controller_active_ = controllerActive;
  }

  void setDtI(double dtI) {
    dt_i = dtI;
  }
  /**
   * Sets Translational stiffness parameters and computes co-stiffness matrix
   * @param K_Pt
   */
  void setK_p_t(Eigen::Vector3d K_Pt) {
    _K_t = K_Pt / _kMass;
    _G_t = _K_t.asDiagonal();
    _G_t = 0.5 * _G_t.trace() * Eigen::Matrix3d::Identity() - _G_t;
  }
  /**
   * Sets Rotational  stiffness parameters and computes co-stiffness matrix
   * @param K_Pr
   */
  void setK_p_r(Eigen::Vector3d K_Pr) {
    _K_r = K_Pr.cwiseQuotient(_kInertiaDiag);
    _G_r = _K_r.asDiagonal();
    _G_r = 0.5 * _G_r.trace() * Eigen::Matrix3d::Identity() - _G_r;
  }

  /**
   * Sets Unnormalized disturbance observer gains
   * Normalization of gains by mass and inertia matrix is unnecessary and leads to instability in case
   * of disturbance rejection
   * @param distObsTransGain0
   * @param distObsRotGain0
   */
  void setDisturbanceObserverGains(double distObsTransGain0, double distObsRotGain0) {
    _K0_obs.topLeftCorner(3, 3) = distObsRotGain0 * Eigen::Matrix3d::Identity();
    _K0_obs.bottomRightCorner(3, 3) = distObsTransGain0 * Eigen::Matrix3d::Identity();
  }
  /**
   * Set Proportional wrench tracking controller gains
   * @param kpW_tau: Rotational gains
   * @param kpW_f: Translational gains
   */
  void setKpW(const Eigen::Vector3d& kpW_tau, const Eigen::Vector3d& kpW_f) {
    _Kp_w.head(3) = kpW_tau;
    _Kp_w.tail(3) = kpW_f;

  }
  /**
   * Set Integral  wrench tracking controller gains
   * @param kiW_tau: Rotational gains
   * @param kiW_f: Translational gains
   */
  void setKiW(const Eigen::Vector3d& kiW_tau, const Eigen::Vector3d& kiW_f) {
    _Ki_w.head(3) = kiW_tau;
    _Ki_w.tail(3) = kiW_f;
  }
  /**
   *
   * @param f_x
   */
  void setCommandWrenchForceX(double f_x) {
    // Convert Desired Force to Body frame as a reaction force (-ve sign)
    W_cmd_B(3) = -f_x;
  }
  /**
   *
   * @param onboardAngularRateGain
   */
  void setOnboardAngularRateGain(const Eigen::Vector3d& onboardAngularRateGain) {
    _kOnboardAngularRateGain = onboardAngularRateGain;
  }
  /***************************************************************************************************************************************/
  /******************************************************* Getters ****************************************************************/

  /**
   *
   * @return
   */
  const Eigen::Vector3d& getAngularRateGain() const {
    return _kAngularRateGain;
  }
  /**
   *
   * @return
   */
  const Eigen::Vector3d& getOnboardAngularRateGain() const {
    return _kOnboardAngularRateGain;
  }
  /**
   *
   * @return
   */
  Eigen::Matrix<double, 6, 1> getEstimatedExternalWrench() {
    return W_hat_ext_B;
  }
  /**
   *
   * @return
   */
  Eigen::Matrix<double, 6, 1> getWrenchTrackingError() {
    return W_err;
  }
  /**
   *
   * @return
   */
  EnergyTank::EnergyTank& getTank1() {
    return tank_1;
  }
  /**
   *
   * @return
   */
  EnergyTank::EnergyTank& getTank2() {
    return tank_2;
  }

  /***************************************************************************************************************************************/
  /**************************************************************** Switches *************************************************/
  /**
   *
   * @param trackEndEffector
   */
  void enableEndEffectorTrack(bool enable_end_effector_track) {
    _enable_end_effector_track = enable_end_effector_track;
  }
  /**
   *
   * @param enable_dist_obs
   */
  void enableDisturbanceObserver(bool enable_dist_obs) {
    _enable_dist_obs = enable_dist_obs;
  }
  /**
   *
   * @param enable_wrench_track
   */
  void enableWrenchTracking(bool enable_wrench_track) {
    _enable_wrench_track = enable_wrench_track;
  }
  /**
   *
   * @param enableEnergyTank
   */
  void setEnableEnergyTank(bool enableEnergyTank) {
    _enable_energy_tank = enableEnergyTank;
  }
  /**
   *
   * @param enableDistReject
   */
  void setEnableDistReject(bool enableDistReject) {
    _enable_dist_reject = enableDistReject;
  }
  /**
   *
   * @return controller state
   */
  bool isControllerActive() const {
    return controller_active_;
  }
  /**
   *
   * @return
   */
  bool isTrackEndEffector() const {
    return _enable_end_effector_track;
  }
  /**
   *
   * @return
   */
  bool isEnableDistReject() const {
    return _enable_dist_reject;
  }

  /***************************************************************************************************************************************/
  /***************************************************************************************************************************************/
 private:
  // controller's states
  bool controller_active_;
  bool _enable_end_effector_track;
  bool _enable_dist_obs;
  bool _enable_dist_reject;
  bool _enable_wrench_track;
  bool _enable_energy_tank;
  bool _distObsRunning;

  // Flight Mode set from GUI
  double flightMode;
  // Gravitational constant
  const double _kGravity = 9.81;
  // UAV's mass [Kg]
  double _kMass;

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

  // Virtual Spring translational stiffness diagonal terms
  Eigen::Vector3d _K_t;
  // Virtual Spring rotational stiffness diagonal terms
  Eigen::Vector3d _K_r;

  // Virtual Spring Costiffness matrices
  Eigen::Matrix<double, 3, 3> _G_t;
  Eigen::Matrix<double, 3, 3> _G_r;
  Eigen::Matrix<double, 3, 3> _G_c;

  // Disturbance Observer Gain Matrix 0
  Eigen::Matrix<double, 6, 6> _K0_obs;
  // Disturbance Observer Auxilary state 1 ( Estimated Momentum)
  Eigen::Matrix<double, 6, 1> P_hat;
  // Disturbance Observer Estimated Wrench
  Eigen::Matrix<double, 6, 1> W_hat_ext_B;

  // Estimated Disturbance Offset Wrench
  Eigen::Matrix<double, 6, 1> W_ext_offset_B;

  // Wrench Regulator Proportional Gain Matrix (diagonal elements)
  Eigen::VectorXd _Kp_w;
  Eigen::VectorXd _Ki_w;

  // Commanded Wrench to Wrench Regulator
  Eigen::Matrix<double, 6, 1> W_cmd_B;

  // Wrench tracking error
  Eigen::Matrix<double, 6, 1> W_err;
  Eigen::Matrix<double, 6, 1> W_err_int;

  // Energy Tanks
  EnergyTank::EnergyTank tank_1;
  EnergyTank::EnergyTank tank_2;

  // Sample Time (Elapsed)
  double dt_i;

};
}

#endif /* INCLUDE_SPC_UAV_CONTROL_ETANK_IMPEDANCE_CONTROLLER_H_ */
