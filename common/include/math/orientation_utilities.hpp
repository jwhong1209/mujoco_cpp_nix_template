/**
 * @file orientation_utilities.hpp
 * @brief Utility functions for 3D rotations
 * @note This file is originated from LIBBIOMIMETICS_ORIENTATION_TOOLS_H
 */

#ifndef ORIENTATION_UTILITIES_HPP_
#define ORIENTATION_UTILITIES_HPP_

#include <cmath>
#include <iostream>
#include <type_traits>

#include "eigen_types.hpp"

namespace orient_utils
{
static constexpr double quaternionDerviativeStabilization = 0.1;

/**
 * @brief Convert radians to degrees
 */
template <typename T>
T rad2deg(T rad)
{
  static_assert(std::is_floating_point<T>::value, "must use floating point value");
  return rad * T(180) / T(M_PI);
}

/**
 * @brief Convert degrees to radians
 */
template <typename T>
T deg2rad(T deg)
{
  static_assert(std::is_floating_point<T>::value, "must use floating point value");
  return deg * T(M_PI) / T(180);
}

/**
 * @brief Convert a quaternion to Euler angles (Roll-Pitch-Yaw)
 * @param q : unit quaternion s.t. s = ||q||^{-2} = 1
 */
template <typename T>
Vec3<T> quaternionToRPY(const Quat<T> & q, const Vec3<T> & rpy_prev)
{
  Quat<T> q_normal = q.normalized();
  T x = q_normal.coeffs()[0];
  T y = q_normal.coeffs()[1];
  T z = q_normal.coeffs()[2];
  T w = q_normal.coeffs()[3];

  T xx = x * x, yy = y * y, zz = z * z;
  T xy = x * y, yz = y * z, zx = z * x;
  T wx = w * x, wy = w * y, wz = w * z;

  Vec3<T> rpy;
  T as_in = std::clamp(2 * (wy - zx), -1.0, 1.0);
  rpy[2] = std::atan2(2 * (wz + xy), 1 - 2 * (yy + zz));  // yaw
  rpy[1] = std::asin(as_in);                              // pitch
  rpy[0] = std::atan2(2 * (wx + yz), 1 - 2 * (xx + yy));  // roll

  if (std::abs(rpy[0] - rpy_prev[0]) > M_PI)
  {
    rpy[0] += (rpy[0] > rpy_prev[0]) ? -2 * M_PI : 2 * M_PI;
  }
  if (std::abs(rpy[1] - rpy_prev[1]) > M_PI_2)
  {
    rpy[1] += (rpy[1] > rpy_prev[1]) ? -M_PI : M_PI;
  }
  if (std::abs(rpy[2] - rpy_prev[2]) > M_PI)
  {
    rpy[2] += (rpy[2] > rpy_prev[2]) ? -2 * M_PI : 2 * M_PI;
  }

  return rpy;
}

/**
 * @brief Convert a quaternion to a rotation matrix. This matrix represents a
 * coordinate transformation into the frame which has the orientation specified
 * by the quaternion
 * @param q : unit quaternion s.t. s = ||q||^{-2} = 1
 */
template <typename T>
RotMat<T> quaternionToRotationMatrix(const Quat<T> & q)
{
  Quat<T> q_normal = q.normalized();
  T x = q_normal.coeffs()[0];
  T y = q_normal.coeffs()[1];
  T z = q_normal.coeffs()[2];
  T w = q_normal.coeffs()[3];

  T xx = x * x, yy = y * y, zz = z * z;
  T xy = x * y, yz = y * z, zx = z * x;
  T wx = w * x, wy = w * y, wz = w * z;

  RotMat<T> R;

  R << 1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (zx + wy),  //
    2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx),     //
    2 * (zx - wy), 2 * (yz + wx), 1 - 2 * (xx + yy);

  return R;
}

/**
 * @brief Compute time-derivative of quaternion
 * @param q : current quaternion; w + xi + yj + zk
 * @param omega : angular velocity expressed in "body" frame
 * @return time-derivative of quaternion
 */
template <typename T1, typename T2>
Quat<T1> getQuatDerivative(const Quat<T1> & q, const Vec3<T2> & omega)
{
  Quat<T1> q_normal = q.normalized();
  T1 x = q_normal.coeffs()[0];
  T1 y = q_normal.coeffs()[1];
  T1 z = q_normal.coeffs()[2];
  T1 w = q_normal.coeffs()[3];

  Mat4<T1> Q;
  Q << w, -x, -y, -z,  //
    x, w, -z, y,       //
    y, z, w, -x,       //
    z, -y, x, w;

  /* transform angular velocity from {W} to {B} */
  // ! CAUTION: omega should be represented in body frame to obtain quaternion derivative
  RotMat<T1> R_wb = quaternionToRotationMatrix(q);  // Rotation Matrix of {B} w.r.t {W}
  Vec3<T2> omega_b = R_wb.transpose() * omega;      // body frame angular velocity

  T2 w_x = omega_b[0];
  T2 w_y = omega_b[1];
  T2 w_z = omega_b[2];

  Vec4<T2> omega_b_vec(0.0, w_x, w_y, w_z);
  Vec4<T1> q_dot = 0.5 * Q * omega_b_vec;

  Quat<T1> dq;
  dq.coeffs() << q_dot[1], q_dot[2], q_dot[3], q_dot[0];  // x, y, z, w

  return dq;
}

/**
 * @brief Compute new quaternion based on angular velocity
 * @param q : current quaternion; w + xi + yj + zk
 * @param omega : angular velocity expressed in "body" frame
 * @param dt : timestep
 * @return new quaternion
 */
template <typename T1, typename T2, typename T3>
Quat<T1> getNewQuat(const Quat<T1> & q, const Vec3<T2> & omega, const T3 dt)
{
  Quat<T1> dq = getQuatDerivative(q, omega);

  Quat<T1> q_new;
  q_new.coeffs() = q.coeffs() + dq.coeffs() * dt;
  q_new.normalize();

  /* Prevent case for opposite direction */
  // if (q.coeffs().dot(q_new.coeffs()) < 0.0)
  // {
  //   q_new.coeffs() << -q_new.coeffs();
  // }

  return q_new;
}

}  // namespace orient_utils

#endif  // ORIENTATION_UTILITIES_HPP_