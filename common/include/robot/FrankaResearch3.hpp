#ifndef FRANKA_RESEARCH3_HPP_
#define FRANKA_RESEARCH3_HPP_

#include "eigen_types.hpp"

template <typename T>
class FrankaResearch3
{
private:
  Vec7<T> q_;    // joint pos
  Vec7<T> dq_;   // joint vel
  Vec7<T> ddq_;  // joint acc

  Vec3<T> p_;   // Cartesian position
  Vec6<T> v_;   // Cartesian velocity
  Vec6<T> a_;   // Cartesian acceleration
  Mat67<T> J_;  // Jacobian matrix

  Mat7<T> M_;      // inertia matrix
  Vec7<T> tau_c_;  // coriolis
  Vec7<T> tau_g_;  // gravity

public:
  FrankaResearch3();

  //* ----- SETTERS --------------------------------------------------------------------------------
  void setJointStates(const Vec7<T> & q, const Vec7<T> & dq);

  //* ----- GETTERS --------------------------------------------------------------------------------
  /* Kinematics */
  Vec3<T> position();
  Vec6<T> velocity();
  Mat67<T> jacobian();

  /* Dynamics */
  Mat6<T> inertia();
  Vec7<T> coriolis();
  Vec7<T> gravity();

  //* ----- PRINTER --------------------------------------------------------------------------------
};

#endif  // FRANKA_RESEARCH3_HPP_