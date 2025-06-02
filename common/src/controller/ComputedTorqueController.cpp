#include "ComputedTorqueController.hpp"

/* C++ STL */
#include <iostream>

/* Dependencies */
#include <Eigen/QR>

/* Custom Libraries */
#include "orientation_utilities.hpp"
#include "robotics_utilities.hpp"

using namespace std;

namespace ori = orient_utils;
namespace robo = robot_utils;

template <typename T>
ComputedTorqueController<T>::ComputedTorqueController()
{
  /* initialize states */
  q_mes_.setZero();
  dq_mes_.setZero();
  q_null_des_ << 0.0, -0.7854, 0.0, -2.5, 0.0, 1.75, -0.754;  // from keyframe
  ee_pos_init_.setZero();
  ee_pos_des_ << 0.322141, 5.44356e-19, 0.434246;  // for initial pose
  ee_pos_cal_.setZero();
  ee_pos_mes_.setZero();
  ee_quat_des_.coeffs() << 0.718008, 0.69581, 0.0127101, 0.0123171;  // for initial pose
  ee_quat_cal_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  ee_quat_mes_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  ee_vel_des_.setZero();
  ee_vel_cal_.setZero();
  ee_vel_mes_.setZero();
  ee_acc_des_.setZero();
  ee_acc_mes_.setZero();
  tau_des_.setZero();

  /* set control parameters */
  K_task_ << 10.0, 10.0, 10.0, 1.0, 1.0, 1.0;
  D_task_ = 2.0 * K_task_.array().sqrt();  // critical damping
  K_null_ << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
  D_null_ = 2.0 * K_null_.array().sqrt();

  /* Define controller modules */
  imp_ = std::make_unique<ImpedanceController<T>>();

  /* create model and planner objects */
  std::string root_path = PROJECT_ROOT_DIR;
  const std::string urdf_filename = root_path + "/assets/model/urdf/fr3_franka_hand.urdf";
  const std::string ee_frame = "fr3_hand_tcp";
  robot_ = std::make_unique<FrankaResearch3Model<T>>(urdf_filename, ee_frame);
  if (!robot_)
  {
    cout << "Error: FR3 Robot Model is not created!" << endl;
  }
  else
  {
    cout << "FR3 Robot Model is created" << endl;
    robot_->updateKinematics(q_mes_, dq_mes_);
  }

  /* create logger object */
  std::string log_filename = root_path + "/assets/data/data.csv";
  logger_ = std::make_unique<SaveData<T>>(log_filename);

  cout << "ComputedTorqueController (CTC) object is created and initialized" << endl;
}

template <typename T>
void ComputedTorqueController<T>::update(const mjModel * m, mjData * d)
{
  getInstance().updateImpl(m, d);
}

template <typename T>
void ComputedTorqueController<T>::updateImpl(const mjModel * m, mjData * d)
{
  dof_ = m->nv - 2;  // degree of freedom (minus 2 is for fingers)
  loop_time_prev_ = loop_time_;
  loop_time_ = d->time;
  T dt = loop_time_ - loop_time_prev_;  // timestep

  //* update states *//
  this->getSensorData(m, d);

  /* compute kinematics */
  robot_->updateKinematics(q_mes_, dq_mes_);
  ee_pos_cal_ = robot_->position();
  ee_quat_cal_ = robot_->quaternion();

  Mat67<T> J = robot_->jacobian();
  Mat76<T> J_t = J.transpose();  // Jacobian transpose
  Mat76<T> J_pinv = J.completeOrthogonalDecomposition().pseudoInverse();
  Mat67<T> J_t_pinv = J_t.completeOrthogonalDecomposition().pseudoInverse();
  ee_vel_cal_ = robot_->velocity();

  /* compute dynamics */
  Mat7<T> M = robot_->inertia();
  Mat6<T> Mo = J_t_pinv * M * J_pinv;  // operational-space inertia
  if (!Mo.allFinite())
  {
    std::cerr << "Warning: Mo contains NaN or Inf !" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  Vec7<T> tau_c = robot_->coriolis();
  Vec7<T> tau_g = robot_->gravity();

  //* set desired trajectory *//
  const T traj_start_time(3);  // trajectory starts at 3 sec
  const T traj_end_time(9);    // trajectory ends at 9 sec
  const bool b_traj_running_ = (traj_start_time <= loop_time_ && loop_time_ < traj_end_time);

  if (b_traj_running_)
  {
    const T radius = 0.05;                                 // [m]
    const T total_time = traj_end_time - traj_start_time;  //
    const T total_angle = 4.0 * M_PI;                      // rotate 2 times
    const T omega = total_angle / total_time;              // [rad/s]
    const T theta = omega * (loop_time_ - traj_start_time);

    ee_vel_des_(0) = -radius * omega * sin(theta);  // vx
    ee_vel_des_(1) = radius * omega * cos(theta);   // vy
    ee_vel_des_(2) = 0.0;                           // vz
    ee_vel_des_.tail(3).setZero();

    ee_acc_des_(0) = -radius * omega * omega * cos(theta);  // ax
    ee_acc_des_(1) = -radius * omega * omega * sin(theta);  // ay
    ee_acc_des_(2) = 0.0;                                   // az
    ee_acc_des_.tail(3).setZero();
  }
  else
  {
    ee_vel_des_.setZero();
    ee_acc_des_.setZero();
  }

  Vec3<T> v_des = ee_vel_des_.head(3);
  Vec3<T> w_des = ee_vel_des_.tail(3);
  ee_pos_des_ += v_des * dt;
  ee_quat_des_ = ori::getNewQuat(ee_quat_des_, w_des, dt);

  //* control law *//
  imp_->setParameters(K_task_, D_task_, K_null_, D_null_);
  Vec6<T> ee_pose_err = imp_->getPoseError(ee_pos_des_, ee_pos_cal_, ee_quat_des_, ee_quat_cal_);
  Vec6<T> ee_pose_err_prev = imp_->getPreviousPoseError();
  Vec6<T> dpose_err = robo::getEulerDerivative(ee_pose_err, ee_pose_err_prev, dt);
  F_imp_ = imp_->getTaskControlForce(dpose_err);

  Vec6<T> Fc = Mo * ee_acc_des_ + F_imp_;
  tau_des_ = J_t * Fc;

  imp_->computeNullConfigError(q_null_des_, q_mes_);
  Vec7<T> tau_null = imp_->getNullControlTorque(dq_mes_, J_t, J_t_pinv);
  tau_des_ += tau_null;

  /* joint-space control */
  tau_des_ += tau_c + tau_g;

  //* send command *//
  for (int i = 0; i < dof_; ++i)
  {
    d->ctrl[i] = tau_des_[i];
  }
  iter_++;
}

/**
 * @brief Get MuJoCo sensor data in MJCF into local member variables
 */
template <typename T>
void ComputedTorqueController<T>::getSensorData(const mjModel * m, mjData * d)
{
  for (int i = 0; i < dof_; ++i)
  {
    q_mes_(i) = d->sensordata[i];
    dq_mes_(i) = d->sensordata[i + dof_];
  }

  ee_quat_mes_.coeffs()[0] = d->sensordata[18];  // x
  ee_quat_mes_.coeffs()[1] = d->sensordata[19];  // y
  ee_quat_mes_.coeffs()[2] = d->sensordata[20];  // z
  ee_quat_mes_.coeffs()[3] = d->sensordata[17];  // w
  // cout << "quat:\t" << ee_quat_mes_.coeffs() << endl;

  for (int i = 0; i < 3; ++i)
  {
    ee_pos_mes_(i) = d->sensordata[i + 14];
    ee_vel_mes_(i) = d->sensordata[i + 21];
    ee_vel_mes_(i + 3) = d->sensordata[i + 24];
    ee_acc_mes_(i) = d->sensordata[i + 27];
    ee_acc_mes_(i + 3) = d->sensordata[i + 30];
  }
  // cout << "pos:\t" << ee_pos_mes_.transpose() << endl;
}

//* ----- LOGGING ----------------------------------------------------------------------------------
template <typename T>
void ComputedTorqueController<T>::startLogging()
{
  if (!b_logging_running_)
  {
    b_logging_running_ = true;
    logging_thread_ = std::thread(&ComputedTorqueController<T>::dataLoggingLoop, this);
  }
}

template <typename T>
void ComputedTorqueController<T>::stopLogging()
{
  if (b_logging_running_)
  {
    b_logging_running_ = false;
    if (logging_thread_.joinable())
    {
      logging_thread_.join();
    }
  }
}

template <typename T>
void ComputedTorqueController<T>::dataLoggingLoop()
{
  while (b_logging_running_)
  {
    {
      std::lock_guard<std::mutex> lock(logging_mtx_);
      logger_->save_scalar(loop_time_);
      logger_->save_vector(ee_pos_des_);
      logger_->save_vector(ee_pos_cal_);
      logger_->save_vector(tau_des_, true);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));  // log data in 100Hz
  }
}

// template class ComputedTorqueController<float>; // ! float type conflict with pinocchio API
template class ComputedTorqueController<double>;