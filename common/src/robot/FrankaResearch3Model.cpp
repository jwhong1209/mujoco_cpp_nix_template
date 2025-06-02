#include "FrankaResearch3Model.hpp"

/* C++ STL */
#include <cmath>

using namespace std;
namespace pin = pinocchio;

template <typename T>
FrankaResearch3Model<T>::FrankaResearch3Model(std::string urdf_filename, std::string ee_framename)
  : urdf_filename_(urdf_filename), ee_framename_(ee_framename)
{
  try
  {
    pin::urdf::buildModel(urdf_filename_, model_);
    data_ = pin::Data(model_);
    ee_frame_id_ = model_.getFrameId(ee_framename_, pin::FrameType::BODY);
  }
  catch (const std::exception & e)
  {
    throw std::runtime_error("Failed to initialize FrankaResearch3Model: " + std::string(e.what()));
  }
}

//* ----- SETTERS ----------------------------------------------------------------------------------
template <typename T>
void FrankaResearch3Model<T>::updateKinematics(const Vec7<T> & q, const Vec7<T> & dq)
{
  /* update joint states */
  q_ = q;
  dq_ = dq;

  /* update kinemaitcs & frames */
  pin::forwardKinematics(model_, data_, q_, dq_);
  pin::updateFramePlacements(model_, data_);
  pin::computeJointJacobians(model_, data_, q_);
  // pin::framesForwardKinematics(model_, data_); // ! check this out

  /* compute Jacobian */
  pin::getFrameJacobian(model_, data_, ee_frame_id_, pin::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_);
}

//* ----- GETTERS ----------------------------------------------------------------------------------
template <typename T>
Vec7<T> FrankaResearch3Model<T>::ddq(const Vec7<T> & tau)
{
  pin::aba(model_, data_, q_, dq_, tau);
  return data_.ddq;
}

/* Kinematics */
template <typename T>
Vec3<T> FrankaResearch3Model<T>::position()
{
  return data_.oMf[ee_frame_id_].translation();
}

template <typename T>
Quat<T> FrankaResearch3Model<T>::quaternion()
{
  return Quat<T>(data_.oMf[ee_frame_id_].rotation());
}

template <typename T>
Vec6<T> FrankaResearch3Model<T>::velocity()
{
  return J_ * dq_;
}

template <typename T>
Mat67<T> FrankaResearch3Model<T>::jacobian()
{
  return J_;
}

/* Dynamics */
template <typename T>
Mat7<T> FrankaResearch3Model<T>::inertia()
{
  pin::crba(model_, data_, q_);
  for (int i = 0; i < model_.nq; ++i)
    for (int j = 0; j < i; ++j)
      data_.M(i, j) = data_.M(j, i);
  return data_.M;
}

template <typename T>
Vec7<T> FrankaResearch3Model<T>::coriolis()
{
  pin::computeCoriolisMatrix(model_, data_, q_, dq_);
  return (data_.C * dq_);
}

template <typename T>
Vec7<T> FrankaResearch3Model<T>::gravity()
{
  pin::computeGeneralizedGravity(model_, data_, q_);
  return data_.g;
}

// template class FrankaResearch3Model<float>;
template class FrankaResearch3Model<double>;