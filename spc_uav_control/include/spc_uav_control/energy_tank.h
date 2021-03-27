/*
 * energy_tank.h
 *
 *  Created on: Aug 26, 2018
 *      Author: ramy
 */

#ifndef INCLUDE_SPC_UAV_CONTROL_ENERGY_TANK_H_
#define INCLUDE_SPC_UAV_CONTROL_ENERGY_TANK_H_

#include <Eigen/Eigen>

namespace EnergyTank {
class EnergyTank {
 public:
  /**
   *  Constructor
   */
  EnergyTank();
  /**
   * Destructor
   */
  ~EnergyTank();

  /**
   *  Evaluated at each sample to check the tank level and set the output of the tank
   * @param w_bar: output from i-th connected subsytem
   * @param port: Used for multiport Energy Tanks i \in {1,2}
   * @return Output from Energy Tank
   */
  Eigen::Matrix<double, 6, 1> connectTank(Eigen::Matrix<double, 6, 1> w_bar, double port);
  /**
   * Fills in tank with energy from connected subsytems
   * @param dt_i: sample time
   * @param D_x1: Damping Energy from subsytem 1
   * @param dH_dx1: output of subsystem 1
   * @param D_x2: Damping Energy from subsytem 2
   * @param dH_dx2:  output of subsystem 2
   */
  void updateTank(double dt_i, double D_x1, Eigen::Matrix<double, 6, 1> dH_dx1, double D_x2,
                  Eigen::Matrix<double, 6, 1> dH_dx2);
  /**
   * Initializes Energy Tank and computes corresponsind state
   * @param T_init: Initial energy to load tank with
   */
  void initTank(double T_init) {
    if (T_init > 0.0) {
      _T_xt = T_init;
      _xt = sqrt(2.0 * T_init);
    }
  }
  /**
   *
   * @param max: Maximum allowed tank energy
   */
  void setMax(double max) {
    _T_max = max;
  }
  /**
   *
   * @param min: : Minimum allowed tank energy
   */
  void setMin(double min) {
    _T_min = min;
  }
  /**
   *
   * @return: Stored tank energy
   */
  double getTankLevel() const {
    return _T_xt;
  }

 private:
  bool _enable_tank;  // Tank Main Switch

  double _T_xt;  // Tank Energy [Joules]
  double _xt;  // Tank associated state

  double _T_max;  // Maximum allowed Energy
  double _T_min;  // Minimum allowed Energy

  double _alpha;  // Low Energy Level switch
  double _beta;  // High Energy Level switch

  Eigen::Matrix<double, 6, 1> _w1;  // 1st  Port
  Eigen::Matrix<double, 6, 1> _w2;  // 2nd Port
  Eigen::Matrix<double, 6, 1> _zero;  // zero vector

};

}

#endif /* INCLUDE_SPC_UAV_CONTROL_ENERGY_TANK_H_ */
