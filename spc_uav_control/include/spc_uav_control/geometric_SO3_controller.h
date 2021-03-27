/*
 * geometric_SO3_controller.h
 *
 *  Created on: Jun 26, 2018
 *      Author: ramy
 */

#ifndef INCLUDE_SPC_UAV_CONTROL_GEOMETRIC_SO3_CONTROLLER_H_
#define INCLUDE_SPC_UAV_CONTROL_GEOMETRIC_SO3_CONTROLLER_H_

#include <Eigen/Eigen>
#include <mav_msgs/eigen_mav_msgs.h>
#include "spc_uav_control/MathHelper.h"

// Define Constants
#define M_PI           3.14159265358979323846  /* pi */
#define FREEFLIGHTMODE 0.0
#define APPROACHMODE   1.0
#define VERTCIRCLEMODE    2.0

namespace motion_controller {

class GeometricSO3Controller {

 public:
  GeometricSO3Controller();
  ~GeometricSO3Controller();
  /**
   * @param T: Thrust force in body-z-axis
   * @param B_z_d: Desired direction of commanded thrust force in body-z-axis
   */
  void computeTrajectoryTracking(double* T, Eigen::Vector3d* B_z_d, Eigen::Matrix<double, 4, 4> H_I_B,
                                 Eigen::Matrix<double, 6, 1> T_B, Eigen::Matrix<double, 4, 4> H_I_C,
                                 Eigen::Matrix<double, 6, 1> T_C, Eigen::Matrix<double, 6, 1> Tdot_C);
  /**
   * @param B_z_d: Desired direction of commanded thrust force in body-z-axis
   * @param tau:  Control torque vector
   */
  void computeAttitudeTracking(const Eigen::Vector3d& B_z_d, Eigen::Vector3d* tau,Eigen::Matrix<double, 4, 4> H_I_B,
                               Eigen::Matrix<double, 6, 1> T_B,
                               Eigen::Matrix<double, 4, 4> H_I_C,
                               Eigen::Matrix<double, 6, 1> T_C,
                               Eigen::Matrix<double, 6, 1> Tdot_C)const;
  /**
   * CONTAINS MOTION CONTROLLER TO TRACK DESIRED CONFIGURATION
   * @param command_wrench: control thrust force and torques for underactuated UAVs
   */
  void calculateCommandWrench(Eigen::VectorXd* command_wrench, Eigen::Matrix<double, 4, 4> H_I_B,
                              Eigen::Matrix<double, 6, 1> T_B, Eigen::Matrix<double, 4, 4> H_I_C,
                              Eigen::Matrix<double, 6, 1> T_C, Eigen::Matrix<double, 6, 1> Tdot_C);
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

 private:
  // controller's state
  bool controller_active_;
  // NOT USED IN CURRENT VERSION
  double flightMode;
  // Gravitational constant
  double _kGravity;
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
  // Controller ANgular velocity Gains
  Eigen::Vector3d _kAngularRateGain;

  double dt_i;

};

}

#endif /* INCLUDE_SPC_UAV_CONTROL_GEOMETRIC_SO3_CONTROLLER_H_ */
