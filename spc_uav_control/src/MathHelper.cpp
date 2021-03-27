#include "spc_uav_control/MathHelper.h"

/**
 * @author  J.J. Graat <j.j.graat@student.utwente.nl>
 *
 * @section DESCRIPTION
 *
 * This namespace contains the Mathmatical functions needed in the RCC_setpoint node.
 *
 */


namespace MathHelper {

  /**
   * <Calculate Skew-symmetric matrix>
   *
   * <Returns the skew-symmetric (or tilde form) of the input vector. The tilde form is
   * defined such that: A x B = tilde(A)*B>
   *
   * @param  Eigen::Vector3d a 
   * @return Eigen::Matrix3d skew symmetric form of vector a
   */
  // ----- Skew ----- //
  Eigen::Matrix3d skew(Eigen::Vector3d a)
  {
    Eigen::Matrix3d a_tilde;
    a_tilde.setZero();
    a_tilde(0,1) = -a(2);
    a_tilde(0,2) = a(1);
    a_tilde(1,0) = a(2);
    a_tilde(1,2) = -a(0);
    a_tilde(2,0) = - a(1);
    a_tilde(2,1) = a(0);
    return a_tilde;
  }

  // tilde operation for twists (By Asem Khattab)
  Eigen::Matrix<double, 4, 4> skewT(Eigen::Matrix<double, 6, 1> a)
  {
    Eigen::Matrix<double, 4, 4> a_tilde;
    a_tilde.setZero();
    a_tilde.topLeftCorner(3, 3) = skew(a.topLeftCorner(3, 1));
    a_tilde.topRightCorner(3, 1) = a.bottomLeftCorner(3, 1);
    return a_tilde;
  }


  /**
   * <Calculate Vector corresponding to Skew-symmetric matrix>
   *
   */
  // ----- Unskew ----- //
  Eigen::Vector3d unskew(Eigen::Matrix3d a_tilde)
  {

    Eigen::Vector3d a;

    a(0) = a_tilde(2,1) ;
    a(1) = a_tilde(0,2);
    a(2) = a_tilde(1,0);

    return a;
  }

  // unskew for twists (By Asem Khattab)
  Eigen::Matrix<double, 6, 1> unskewT(Eigen::Matrix<double, 4, 4> a_tilde)
  {
    Eigen::Matrix<double, 6, 1> a;
    a.topLeftCorner(3, 1) = unskew(a_tilde.topLeftCorner(3, 3));
    a.bottomLeftCorner(3, 1) = a_tilde.topRightCorner(3, 1);
    return a;
  }

  /**
   * <Calculate anti-symmetric matrix>
   *
   * <Returns an anti-symmetric matrix. This function is equal to: 
   * y = (A - transpose(A))/2>
   *
   * @param  Eigen::Matrix3d a 
   * @return Eigen::Matrix3d anti-symmetric matrix
   */
  // ----- Antisymmetric ----- //
  Eigen::Matrix3d antisym(Eigen::Matrix3d a)
  {
    Eigen::Matrix3d b;
    b = (a-a.transpose())/2;
    return b;
  }



  /**
   * <Calculate the Adjoint of an homogeneous matrix>
   *
   * <Returns the (big) Adjoint (6x6) of an homogeneous matrix. 
   * The Adjoint can be used to transform a geometrical Twist or Wrench>
   *
   * @param  Eigen::Matrix4d H homogeneous matrix
   * @return Eigen::Matrix6d Adjoint of H
   */
  // ----- Adjoint ----- //
  Eigen::Matrix<double,6,6> Adjoint(Eigen::Matrix<double,4,4> H_in)
  {
    Eigen::Matrix<double,6,6> Adjoint_H;
    Adjoint_H.setZero();
    Eigen::Matrix3d rotation = H_in.topLeftCorner(3, 3);
    Eigen::Vector3d position = H_in.topRightCorner(3, 1);

    Adjoint_H.topLeftCorner(3,3) = rotation;
    Adjoint_H.bottomRightCorner(3,3) = rotation;
    Adjoint_H.bottomLeftCorner(3,3) = skew(position)*rotation;

    return Adjoint_H;
  }



  /**
   * <Calculate the inverse of an H-matrix>
   *
   * <Returns the inverse of an H-matrix. One could also just use
   * the 'Eigen::inverse()' function, but the inverse of an 
   * homogeneous H-matrix can be calculated more efficient as has
   * been done in this function.
   *
   * @param  Eigen::Matrix4d H homogenous matrix 
   * @return Eigen::Matrix4d Hinv inverse of H
   */
  // ----- InverseH  ----- //
  Eigen::Matrix4d inverseH(Eigen::Matrix4d H_in)
  {
    Eigen::Matrix4d H_inv;
    H_inv.setIdentity();
    Eigen::Matrix3d R_in = H_in.topLeftCorner(3,3);
    Eigen::Vector3d p_in = H_in.topRightCorner(3,1);
    H_inv.topLeftCorner(3,3) = R_in.transpose();
    H_inv.topRightCorner(3,1) = -1*R_in.transpose()*p_in;

    return H_inv;
  }

/**
 * Compute adjoint representation of se(3)
 * @param T_in: Twist vector
 * @return 6x6 adjoint matrix
 */
  Eigen::Matrix<double,6,6> adjoint(Eigen::Matrix<double,6,1> T_in){
    Eigen::Matrix<double,6,6> adjoint_T;
    adjoint_T.setZero();
       Eigen::Vector3d w = T_in.head(3);
       Eigen::Vector3d v = T_in.tail(3);
       adjoint_T.topLeftCorner(3,3) = skew(w);
       adjoint_T.bottomRightCorner(3,3) =  skew(w);
       adjoint_T.bottomLeftCorner(3,3) =  skew(v);

       return adjoint_T;
  }

  /**
   * implements multidimensional sign operator
   * @param X_in: 6D vector
   * @return 6D sign vector
   */
  Eigen::Matrix<double,6,1> signArray(Eigen::Matrix<double,6,1> X_in){
    Eigen::Matrix<double,6,1> sgn_X;
    sgn_X.setZero();

    for (int i = 0; i < 6; i++) {
      sgn_X(i) = (X_in(i) > 0.0f) - (X_in(i) < 0.0f);
    }

    return sgn_X;
  }

  /**
   * Function to calculate the exponential map  -Rodrigues Formula (By Asem Khattab)
   * @param a
   * @return
   */
  Eigen::Matrix3d expMap(Eigen::Vector3d a){
    double a_norm = a.norm();
    Eigen::Matrix3d expA;
    expA.setIdentity();
    if (a_norm != 0.00){
      Eigen::Matrix3d a_tilde = skew(a);
      expA += std::sin(a_norm)*a_tilde + ((double)1.0 - std::cos(a_norm))*a_tilde*a_tilde;
    }
    return expA;
  }

  /**
   * Function to calculate the Twist exponential map  (By Asem Khattab)
   * @param a
   * @return
   */
  Eigen::Matrix<double, 4, 4> expMapT(Eigen::Matrix<double, 6, 1> a){
    Eigen::Vector3d w = a.topLeftCorner(3, 1);
    Eigen::Vector3d v = a.bottomLeftCorner(3, 1);
    Eigen::Matrix<double, 4, 4> expA;
    expA.setIdentity();

    if (a.topLeftCorner(3, 1).norm() == 0){
      expA.topRightCorner(3, 1) = v;
    }
    else {
      Eigen::Matrix3d expW = expMap(w);
      expA.topLeftCorner(3, 3) = expW;
      expA.topRightCorner(3, 1) = ((Eigen::Matrix3d::Identity() - expW) * (skew(w) * v)
                                   + (w.dot(v)*w)) / w.squaredNorm();
    }
    return expA;
  }


}
