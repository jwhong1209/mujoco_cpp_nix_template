#ifndef FRANKA_RESEARCH3_MODEL_HPP_
#define FRANKA_RESEARCH3_MODEL_HPP_

/* C++ STL */
#include <string>

/* Dependencies */
#include "pinocchio/algorithm/aba.hpp"     // forward dynamics
#include "pinocchio/algorithm/crba.hpp"    // joint-space inertia (upper-triangle)
#include "pinocchio/algorithm/frames.hpp"  // updateFramePlacements
#include "pinocchio/algorithm/rnea.hpp"    // Coriolis, gravity, joint torque
#include "pinocchio/autodiff/casadi.hpp"   // autodiff tools
#include "pinocchio/parsers/urdf.hpp"      // create model from URDF

/* Custom Libraries */
#include "eigen_types.hpp"

template <typename T>
class FrankaResearch3Model
{
private:
  pinocchio::Model model_;
  pinocchio::Data data_;

  std::string urdf_filename_;
  std::string ee_framename_;
  int ee_frame_id_;  // end-effector frame ID

  Vec7<T> q_;   // joint pos
  Vec7<T> dq_;  // joint vel

  Mat67<T> J_;  // Jacobian matrix

public:
  FrankaResearch3Model(std::string urdf_filename, std::string ee_framename);

  //* ----- SETTERS --------------------------------------------------------------------------------
  void updateKinematics(const Vec7<T> & q, const Vec7<T> & dq);

  //* ----- GETTERS --------------------------------------------------------------------------------
  Vec7<T> ddq(const Vec7<T> & tau);

  /* Kinematics */
  Vec3<T> position();
  Quat<T> quaternion();
  Vec6<T> velocity();
  Mat67<T> jacobian();

  /* Dynamics */
  Mat7<T> inertia();
  Vec7<T> coriolis();
  Vec7<T> gravity();

  //* ----- PRINTER --------------------------------------------------------------------------------
};

#endif  // FRANKA_RESEARCH3_MODEL_HPP_