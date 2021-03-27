#ifndef MATHHELPER_H
#define MATHHELPER_H

#include <Eigen/Dense>
#include <Eigen/Geometry> 

namespace MathHelper {

// ----- Skew ----- //
Eigen::Matrix3d skew(Eigen::Vector3d a);
// for twists:
Eigen::Matrix<double, 4, 4> skewT(Eigen::Matrix<double, 6, 1> a);

// ----- Unskew ----- //
Eigen::Vector3d unskew(Eigen::Matrix3d a_tilde);
// for twists:
Eigen::Matrix<double, 6, 1> unskewT(Eigen::Matrix<double, 4, 4> a_tilde);

// ----- Antisymmetric ----- //
Eigen::Matrix3d antisym(Eigen::Matrix3d a);

// ----- Lie Group Adjoint ----- //
Eigen::Matrix<double, 6, 6> Adjoint(Eigen::Matrix<double, 4, 4> H_in);

// ----- InverseH  ----- //
Eigen::Matrix4d inverseH(Eigen::Matrix4d H_in);

// ----- Lie AlgebraAdjoint ----- //
Eigen::Matrix<double, 6, 6> adjoint(Eigen::Matrix<double, 6, 1> T_in);

// ----- sgn function for vector----- //
Eigen::Matrix<double, 6, 1> signArray(Eigen::Matrix<double, 6, 1> X_in);

Eigen::Matrix3d expMap(Eigen::Vector3d a);

Eigen::Matrix<double, 4, 4> expMapT(Eigen::Matrix<double, 6, 1> a);

void test();

}

#endif
