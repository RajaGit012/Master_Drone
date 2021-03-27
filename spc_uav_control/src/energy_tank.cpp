/*
 * energy_tank.cpp
 *
 *  Created on: Aug 26, 2018
 *      Author: ramy
 */

#include "spc_uav_control/energy_tank.h"

namespace EnergyTank {

EnergyTank::EnergyTank() {
  // Turn off Tank and initialize all to zero
  _enable_tank = false;
  _T_min = 0.0;
  _T_max = 0.0;
  _w1.setZero();
  _w2.setZero();
  _zero.setZero();
  _beta = 0.0;
  _alpha=0.0;
  _T_xt = 0.0;
  _xt = 0.0;
}

EnergyTank::~EnergyTank() {

}

Eigen::Matrix<double, 6, 1> EnergyTank::connectTank(Eigen::Matrix<double, 6, 1> w_bar, double port) {
  // Compute tank energy
  _T_xt = 0.5 * _xt * _xt;

  // Check Low Limit
  if (_T_xt >= _T_min) {
    _alpha = 1.0;
  } else {
    _alpha = 0.0;
  }

// Check High Limit
  if (_T_xt <= _T_max) {
    _beta = 1.0;
  } else {
    _beta = 0.0;
  }

  // Connect input to Port 1
  if (port == 1) {
    _w1 = _alpha * w_bar;
    return _w1;

  }

  // Connect input to Port 2
  if (port == 2) {
    _w2 = _alpha * w_bar;
    return _w2;
  }

  // Wrong Port set in parameter
  return _zero;
}

void EnergyTank::updateTank(double dt_i, double D_x1, Eigen::Matrix<double, 6, 1> dH_dx1, double D_x2,
                            Eigen::Matrix<double, 6, 1> dH_dx2) {
  // Compute Tank's input
  double u_t_bar;
  u_t_bar = (_w1.transpose() * dH_dx1);
  u_t_bar += (_w2.transpose() * dH_dx2);

  // Update Tank's state
  _xt += (dt_i / _xt) * (_beta * (D_x1 + D_x2) - u_t_bar);

}

}

