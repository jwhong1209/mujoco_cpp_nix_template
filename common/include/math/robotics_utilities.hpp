/**
 * @file robotics_utilities.hpp
 * @brief Contains methods that are necessary for robotics problem
 * NOTE: Fixed size Eigen::Matrix can not converted to Eigen::Dynamic type
 * - dynamic type that is determined during run-timne cannot be compatible with fixed size type
 * which is determined during compile-time
 */

#ifndef ROBOTICS_UTILITIES_HPP_
#define ROBOTICS_UTILITIES_HPP_

#include <cmath>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>

#include "eigen_types.hpp"

namespace robot_utils
{
//* ----- Filters ----------------------------------------------------------------------------------
template <typename T, int Rows>
auto getFirstLowPassFiltered(const Eigen::Matrix<T, Rows, 1> & u,      //
                             const Eigen::Matrix<T, Rows, 1> & y_old,  //
                             const T & dt, const T & fc = 150)
{
  T tau = 1 / (2 * M_PI * fc);
  return (dt * u + tau * y_old) / (dt + tau);
};

//* ----- Numerical Derivative ---------------------------------------------------------------------
/**
 * @param u : any n x 1 vector e.g. joint pos, vel, acc, ...
 * @param u_old : input values in previous iteration
 * @param dt : timestep
 * @return time-derivative using Backward Euler method
 * NOTE: `eval` is necessary to explicitly evaluate return value
 */
template <typename T, int Rows>
auto getEulerDerivative(const Eigen::Matrix<T, Rows, 1> & u,      //
                        const Eigen::Matrix<T, Rows, 1> & u_old,  //
                        const T & dt, const T & threshold = 1e-6)
{
  Eigen::Matrix<T, Rows, 1> diff = u - u_old;
  // TODO: check noise level
  // if (diff.norm() < threshold)
  // {
  //   return Eigen::Matrix<T, Rows, 1>::Zero().eval();
  // }
  return ((u - u_old) / dt).eval();
}

/**
 * @param u : any n x 1 vector e.g. joint pos, vel, acc, ...
 * @param u_old : input values in previous iteration
 * @param y_old : output values in previous iteration
 * @param dt : timestep
 * @param fc : cutoff frequency [Hz]
 * @return time-derivative using Backward Euler method with Low-pass Filter
 */
template <typename T, int Rows>
auto getFilteredEulerDerivative(const Eigen::Matrix<T, Rows, 1> & u,      //
                                const Eigen::Matrix<T, Rows, 1> & u_old,  //
                                const Eigen::Matrix<T, Rows, 1> & y_old,  //
                                const T & dt, const T & fc = 150)
{
  T tau = 1 / (2 * M_PI * fc);
  return (u - u_old + tau * y_old) / (dt + tau);
};

/**
 * @param u : any n x 1 vector e.g. joint pos, vel, acc, ...
 * @param u_old : input values in previous iteration
 * @param y_old : output values in previous iteration
 * @param dt : timestep
 * @return time-derivative using Tustin (Bilinear) method
 */
template <typename T, int Rows>
auto getTustinDerivative(const Eigen::Matrix<T, Rows, 1> & u,      //
                         const Eigen::Matrix<T, Rows, 1> & u_old,  //
                         const Eigen::Matrix<T, Rows, 1> & y_old, const T & dt)
{
  return 2 * (u - u_old) / dt - y_old;
};

/**
 * @param u : any n x 1 vector e.g. joint pos, vel, acc, ...
 * @param u_old : input values in previous iteration
 * @param y_old : output values in previous iteration
 * @param dt : timestep
 * @param fc : cutoff frequency [Hz]
 * @return time-derivative using Tustin (Bilinear) method with Low-pass Filter
 */
template <typename T, int Rows>
auto getFilteredTustinDerivative(const Eigen::Matrix<T, Rows, 1> & u,      //
                                 const Eigen::Matrix<T, Rows, 1> & u_old,  //
                                 const Eigen::Matrix<T, Rows, 1> & y_old,  //
                                 const T & dt, const T & fc = 150)
{
  T tau = 1 / (2 * M_PI * fc);
  return (2 * (u - u_old) - (dt - 2 * tau) * y_old) / (dt + 2 * tau);
};

/**
 * @brief Compute task-space acceleration
 * @param J : nt x nv Jacobian matrix (nt : task-dof)
 * @param dJ : nt x nv Time-derivative of Jacobian matrix
 * @param dq : nv x 1 Joint velocity
 * @param ddq : nv x 1 Joint acceleration
 */
template <typename T, int Rows, int Cols>
auto getTaskAcceleration(const Eigen::Matrix<T, Rows, Cols> & J,
                         const Eigen::Matrix<T, Rows, Cols> & dJ,
                         const Eigen::Matrix<T, Cols, 1> & dq,
                         const Eigen::Matrix<T, Cols, 1> & ddq)
{
  return J * ddq + dJ * dq;
};

//* ----- Kinematics -------------------------------------------------------------------------------
template <typename T, int Rows, int Cols>
MatX<T> getPseudoInverse(const Eigen::Matrix<T, Rows, Cols> & M, bool damped = true)
{
  T lambda = damped ? static_cast<T>(0.2) : static_cast<T>(0.0);

  Eigen::JacobiSVD<MatX<T>> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
  // NOTE: you should add `typename` cause it's dependent scope
  typename Eigen::JacobiSVD<MatX<T>>::SingularValuesType sing_vals = svd.singularValues();

  MatX<T> S = MatX<T>::Zero(M.rows(), M.cols());
  for (int i = 0; i < sing_vals.size(); i++)
  {
    S(i, i) = sing_vals(i) / (sing_vals(i) * sing_vals(i) + lambda * lambda);
  }

  return svd.matrixV() * S.transpose() * svd.matrixU().transpose();
};

//* ----- Dynamics ---------------------------------------------------------------------------------
/**
 * @param M_ut : upper-triangular inertia matrix
 * @return n x n (full) joint-space inertia matrix
 */
template <typename T, int Rows, int Cols>
auto getDenseInertiaMatrix(const Eigen::Matrix<T, Rows, Cols> & M_ut)
{
  Eigen::Matrix<T, Rows, Cols> M_dense(M_ut.rows(), M_ut.cols());
  M_dense.template triangularView<Eigen::Upper>() = M_ut;
  M_dense.template triangularView<Eigen::Lower>() = M_ut.transpose();
  return M_dense;
}

/**
 * @param M : configuration-space inertia matrix
 * @param J_inv : (pseudo-)inverse of Jacobian matrix
 * @param Jt_inv : (pseudo-)inverse of Jacobian.transpose matrix
 * @return n x n operational inertia matrix
 */
template <typename DerivedM, typename DerivedJInv, typename DerivedJtInv>
auto getTaskInertiaMatrix(const Eigen::MatrixBase<DerivedM> & M,
                          const Eigen::MatrixBase<DerivedJInv> & J_inv,
                          const Eigen::MatrixBase<DerivedJtInv> & Jt_inv)
{
  using Scalar = typename DerivedM::Scalar;
  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Mt(Jt_inv.rows(), J_inv.cols());
  Mt.noalias() = Jt_inv * M * J_inv;  // prevent memory replicate
  return Mt.eval();                   // completely evaluated matrix
}

}  // namespace robot_utils

#endif  // ROBOTICS_UTILITIES_HPP_
