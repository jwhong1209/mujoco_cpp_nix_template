#include "ComputedTorqueController.hpp"

/* C++ STL */
#include <iostream>

/* Dependencies */
#include <Eigen/QR>

using namespace std;

template <typename T>
ComputedTorqueController<T>::ComputedTorqueController()
{
  /* initialize states */
  q_mes_.setZero();
  dq_mes_.setZero();
  p_init_.setZero();
  p_des_.setZero();
  p_cal_.setZero();
  p_mes_.setZero();
  v_des_.setZero();
  v_cal_.setZero();
  v_mes_.setZero();
  a_des_.setZero();
  tau_des_.setZero();

  /* set control parameters */
  K_task_ << 200.0, 200.0, 200.0, 20.0, 20.0, 20.0;
  D_task_ = 2.0 * K_task_.array().sqrt();  // critical damping

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
  loop_time_ = d->time;

  //* update states *//
  this->getSensorData(m, d);
  robot_->updateKinematics(q_mes_, dq_mes_);

  /* compute kinematics */
  p_cal_ = robot_->position();

  Mat67<T> J = robot_->jacobian();
  Mat76<T> J_t = J.transpose();  // Jacobian transpose
  // Mat76<T> J_pinv = J.inverse();
  // Mat67<T> J_t_pinv = J_t.inverse();
  v_cal_ = robot_->velocity();

  /* compute dynamics */
  Mat7<T> M = robot_->inertia();
  if (M.determinant() < 1e-10)
  {
    std::cerr << "Warning: M is nearly singular !" << std::endl;
  }

  // Mat6<T> Mo = J_t_pinv * M * J_pinv;  // operational-space inertia
  // if (!Mo.allFinite())
  // {
  //   std::cerr << "Warning: Mo contains NaN or Inf !" << std::endl;
  //   std::exit(EXIT_FAILURE);
  // }

  Vec7<T> tau_c = robot_->coriolis();
  Vec7<T> tau_g = robot_->gravity();

  //* set desired trajectory *//
  // const T traj_start_time(3);  // trajectory starts at 3 sec
  // const T traj_end_time(9);    // trajectory ends at 9 sec
  // const bool b_traj_running_ = (traj_start_time <= loop_time_ && loop_time_ < traj_end_time);

  //* control law *//
  /* task-space control */
  // Vec6<T> Fc = Mo * a_des_ + K_task_.cwiseProduct(p_des_ - p_mes_) +
  // D_task_.cwiseProduct(-v_mes_); tau_des_ = J_t * Fc;

  /* joint-space control */
  // tau_des_ += tau_g;

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

  // p_mes_(0) = d->sensordata[5];        // y direction
  // p_mes_(1) = d->sensordata[6] - 1.5;  // z direction (1.5 is initial height)

  // v_mes_(0) = d->sensordata[12];  // y direction
  // v_mes_(1) = d->sensordata[13];  // z direction (1.5 is initial height)
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
      logger_->save_vector(p_des_);
      logger_->save_vector(p_cal_);
      logger_->save_vector(tau_des_, true);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));  // log data in 100Hz
  }
}

// template class ComputedTorqueController<float>; // ! float type conflict with pinocchio API
template class ComputedTorqueController<double>;