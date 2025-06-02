#ifndef IMPEDANCE_CONTROLLER_HPP_
#define IMPEDANCE_CONTROLLER_HPP_

#include "eigen_types.hpp"

template <typename T>
class ImpedanceController
{
private:
  Vec6<T> x_0e_err_;       // end-effector 6-DoF pose error
  Vec6<T> x_0e_err_prev_;  // previous error
  Mat6<T> K_task_;         // task-space stiffness matrix
  Mat6<T> D_task_;         // task-space damping matrix
  Vec6<T> F_task_;         // computed task-space wrench from impedance controller
  Vec7<T> tau_task_;       // joint-spce impedance torque

  Vec7<T> q_null_err_;       // null-space joint configuration error
  Vec7<T> q_null_err_prev_;  // previous error
  Mat7<T> K_null_;           // null-space stiffness matrix
  Mat7<T> D_null_;           // null-space damping matrix
  Vec7<T> tau_null_;         // computed null-space torque from impedance controller

public:
  ImpedanceController();

  //* ----- SETTERS --------------------------------------------------------------------------------
  void setParameters(const Vec6<T> & K_task, const Vec6<T> & D_task,  //
                     const Vec7<T> & K_null, const Vec7<T> & D_null);

  //* ----- METHODS --------------------------------------------------------------------------------
  void updatePreviousStates()
  {
    q_null_err_prev_ = q_null_err_;
    x_0e_err_prev_ = x_0e_err_;
  };

  /**
   * @param q_null_des : desired null-space joint configuration
   * @param q_null_mes : measured null-space joint configuration
   */
  void computeNullConfigError(const Vec7<T> & q_null_des, const Vec7<T> & q_null_mes);

  //* ----- GETTERS --------------------------------------------------------------------------------
  /**
   * @param dt : timestep
   * @param p_0e_des : desired end-effector position in {W} frame
   * @param p_0e_mes : measured end-effector position in {W} frame
   * @param q_0e_des : desired end-effector quaternion w.r.t {W} frame
   * @param q_0e_mes : measured end-effector quaternion w.r.t {W} frame
   */
  Vec6<T> getPoseError(const Vec3<T> & p_0e_des, const Vec3<T> & p_0e_mes, const Quat<T> & q_0e_des,
                       const Quat<T> & q_0e_mes);

  Vec6<T> getPreviousPoseError()
  {
    return x_0e_err_prev_;
  };

  /**
   * @param dx_0e_err : Time-derivative of end-effector 6-DoF pose error
   */
  Vec6<T> getTaskControlForce(const Vec6<T> & dx_0e_err)
  {
    return K_task_ * x_0e_err_ + D_task_ * dx_0e_err;
  };

  /**
   * @param dq_null_err : Time-derivative of null-space joint configuration error
   * @param J_t : end-effector Jacobian transpose matrix
   * @param J_t_pinv : pseudo-inverse of Jacobian transpose matrix
   */
  Vec7<T> getNullControlTorque(const Vec7<T> & dq_null_err, const MatX<T> & J_t,
                               const MatX<T> & J_t_pinv);
};

#endif  // IMPEDANCE_CONTROLLER_HPP_