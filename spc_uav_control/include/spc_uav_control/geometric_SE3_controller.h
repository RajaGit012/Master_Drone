/*
 * geometric_SE3_controller.h
 *
 *  Created on: Jan 19, 2018
 *      Author: ramy
 *  Refactored on: May 2018
 *      Author: Asem
 */

#ifndef INCLUDE_SPC_UAV_CONTROL_GEOMETRIC_SE3_CONTROLLER_H_
#define INCLUDE_SPC_UAV_CONTROL_GEOMETRIC_SE3_CONTROLLER_H_

#include <Eigen/Eigen>
#include <mav_msgs/eigen_mav_msgs.h>
#include "spc_uav_control/MathHelper.h"

// Define Constants
#define M_PI           3.14159265358979323846  /* pi */
#define FREEFLIGHTMODE 0.0
#define APPROACHMODE   1.0
#define VERTCIRCLEMODE    2.0

#define NODISTURBANCEOBSERVER 0
#define DISTURBANCEOBSERVER_1   1
#define DISTURBANCEOBSERVER_2    2
#define DISTURBANCEOBSERVER_3    3

namespace motion_controller{
class GeometricSE3Controller {
 public:
  GeometricSE3Controller();
  ~GeometricSE3Controller();
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
  void calculateCommandWrench(Eigen::VectorXd* command_wrench,
                              Eigen::Matrix<double, 4, 4> H_B_E,
                              Eigen::Matrix<double, 4, 4> H_I_B,
                              Eigen::Matrix<double, 6, 1> T_B,
                              Eigen::Matrix<double, 4, 4> H_I_C,
                              Eigen::Matrix<double, 6, 1> T_C,
                              Eigen::Matrix<double, 6, 1> Tdot_C);
    /***************************************************************************************************************************************/

  void setAngularRateGain(const Eigen::Vector3d& angularRateGain) {
    _kAngularRateGain = angularRateGain;
  }
  /**
   *
   * @param attitudeGain
   */
  void setAttitudeGain(const Eigen::Vector3d& attitudeGain) {
    _kAttitudeGain = attitudeGain;
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
   * @param positionGain
   */
  void setPositionGain(const Eigen::Vector3d& positionGain) {
    _kPositionGain = positionGain / _kMass;
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

  void setDistObsGain(double distObsTransGain0,double distObsRotGain0,double distObsTransGain1,double distObsRotGain1) {
    _kDistObsGain_0.topLeftCorner(3, 3) = distObsRotGain0 * Eigen::Matrix3d::Identity();
    _kDistObsGain_0.bottomRightCorner(3, 3) = distObsTransGain0 * Eigen::Matrix3d::Identity();

    _kDistObsGain_1.topLeftCorner(3, 3) = distObsRotGain1 * Eigen::Matrix3d::Identity();
    _kDistObsGain_1.bottomRightCorner(3, 3) = distObsTransGain1 * Eigen::Matrix3d::Identity();
  }

  void setDtI(double dtI) {
    dt_i = dtI;
  }

  void setDisturbanceObserverType(int disturbanceObserverType) {
    disturbance_observer_type = disturbanceObserverType;
  }

  void setDoOn(float doOn) {
    DO_ON = doOn;
  }


 private:
  // controller's state
  bool controller_active_;
  bool _DistObsRunning;
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
  // Controller ANgular velocity Gains
  Eigen::Vector3d _kAngularRateGain;

  // Disturbance Observer Gain Matrix 0
  Eigen::Matrix<double, 6, 6> _kDistObsGain_0;
  // Disturbance Observer Gain Matrix 1
  Eigen::Matrix<double, 6, 6> _kDistObsGain_1;
  // Disturbance Observer Auxilary state 1
  Eigen::Matrix<double, 6, 1> deltaW;
  // Disturbance Observer Auxilary state2
  Eigen::Matrix<double, 6, 1> etaW;

  double dt_i;
  int disturbance_observer_type;
  float DO_ON;
};
}

#endif /* INCLUDE_SPC_UAV_CONTROL_GEOMETRIC_SE3_CONTROLLER_H_ */
