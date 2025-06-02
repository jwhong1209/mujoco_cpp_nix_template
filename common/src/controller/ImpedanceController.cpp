#include "ImpedanceController.hpp"

#include "orientation_utilities.hpp"

#include <iostream>

namespace ori = orient_utils;

using namespace std;

template <typename T>
ImpedanceController<T>::ImpedanceController()
{
  x_0e_err_.setZero();
  x_0e_err_prev_.setZero();
  F_task_.setZero();
  tau_task_.setZero();

  q_null_err_.setZero();
  q_null_err_prev_.setZero();
  tau_null_.setZero();
}

//* ----- SETTERS ----------------------------------------------------------------------------------
template <typename T>
void ImpedanceController<T>::setParameters(const Vec6<T> & K_task, const Vec6<T> & D_task,
                                           const Vec7<T> & K_null, const Vec7<T> & D_null)
{
  K_task_ = K_task.asDiagonal();
  D_task_ = D_task.asDiagonal();
  K_null_ = K_null.asDiagonal();
  D_null_ = D_null.asDiagonal();
}

//* ----- METHODS ----------------------------------------------------------------------------------
template <typename T>
void ImpedanceController<T>::computeNullConfigError(const Vec7<T> & q_null_des,
                                                    const Vec7<T> & q_null_mes)
{
  q_null_err_ = q_null_des - q_null_mes;
}

//* ----- GETTERS ----------------------------------------------------------------------------------
template <typename T>
Vec6<T> ImpedanceController<T>::getPoseError(const Vec3<T> & p_0e_des, const Vec3<T> & p_0e_mes,
                                             const Quat<T> & q_0e_des, const Quat<T> & q_0e_mes)
{
  x_0e_err_.head(3) << p_0e_des - p_0e_mes;

  Quat<T> q_new;
  if (q_0e_des.coeffs().dot(q_0e_mes.coeffs()) < 0.0)
  {
    q_new.coeffs() << -q_0e_mes.coeffs();
  }
  else
  {
    q_new.coeffs() << q_0e_mes.coeffs();
  }
  Quat<T> q_0e_err = q_new.inverse() * q_0e_des;
  RotMat<T> R = ori::quaternionToRotationMatrix(q_new);
  x_0e_err_.tail(3) << q_0e_err.x(), q_0e_err.y(), q_0e_err.z();
  x_0e_err_.tail(3) << R * x_0e_err_.tail(3);

  return x_0e_err_;
}

template <typename T>
Vec7<T> ImpedanceController<T>::getNullControlTorque(const Vec7<T> & dq_null_err,
                                                     const MatX<T> & J_t, const MatX<T> & J_t_pinv)
{
  if (!J_t.allFinite())
  {
    std::cerr << "[Impedance] Warning: J_t contains NaN or Inf !" << std::endl;
  }

  if (!J_t_pinv.allFinite())
  {
    std::cerr << "[Impedance] Warning: J_t_pinv contains NaN or Inf !" << std::endl;
  }

  Mat7<T> I = Mat7<T>::Identity();
  return (I - J_t * J_t_pinv) * (K_null_ * q_null_err_ - D_null_ * dq_null_err);
}

// template class ImpedanceController<float>; // ! float type induces error due to Eigen::MatrixXd
template class ImpedanceController<double>;