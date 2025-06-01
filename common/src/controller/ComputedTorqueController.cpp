#include "ComputedTorqueController.hpp"

#include <iostream>

using namespace std;

template <typename T>
ComputedTorqueController<T>::ComputedTorqueController()
{
  /* initialize states */
  // q_mes_.setZero();
  q_mes_ << -0.7854, 1.5708;
  dq_mes_.setZero();
  p_init_.setZero();
  p_des_.setZero();
  p_cal_.setZero();
  p_mes_.setZero();
  v_des_.setZero();
  v_cal_.setZero();
  v_mes_.setZero();
  a_des_.setZero();
  F_ext_local_.setZero();
  F_ext_world_.setZero();
  tau_des_.setZero();

  /* set control parameters */
  kp_ << 50.0, 50.0;
  kd_ << 5, 5;

  /* create model and planner objects */
  robot_ = std::make_unique<DoublePendulumModel<T>>();
  if (!robot_)
  {
    cout << "Error: Double Pendulum Model is not created!" << endl;
  }
  else
  {
    cout << "Double Pendulum Model is created" << endl;
    robot_->setJointStates(q_mes_, dq_mes_);
  }

  planner_ = std::make_unique<TrajectoryGenerator<T>>();
  if (!planner_)
  {
    cout << "Error: Trajectory Generator is not created!" << endl;
  }
  else
  {
    cout << "Trajectory Generator is created" << endl;
  }

  /* create logger object */
  std::string root = PROJECT_ROOT_DIR;
  std::string log_filename = root + "/assets/data/data.csv";
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
  dof_ = m->nv;  // degree of freedom
  loop_time_ = d->time;

  //* update states *//
  this->getSensorData(m, d);
  robot_->setJointStates(q_mes_, dq_mes_);

  /* compute kinematics */
  p_cal_ = robot_->position();

  Mat2<T> J = robot_->jacobian();
  Mat2<T> J_t = J.transpose();  // Jacobian transpose
  Mat2<T> J_inv = J.inverse();
  Mat2<T> J_t_inv = J_t.inverse();
  v_cal_ = robot_->velocity();

  /* compute dynamics */
  Mat2<T> M = robot_->inertia();
  if (M.determinant() < 1e-10)
  {
    std::cerr << "Warning: M is nearly singular !" << std::endl;
  }

  Mat2<T> Mo = J_t_inv * M * J_inv;  // operational-space inertia
  if (!Mo.allFinite())
  {
    std::cerr << "Warning: Mo contains NaN or Inf !" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  Vec2<T> tau_c = robot_->coriolis();
  Vec2<T> tau_g = robot_->gravity();

  //* set desired trajectory *//
  const T traj_start_time(3);  // trajectory starts at 3 sec
  const T traj_end_time(9);    // trajectory ends at 9 sec
  const bool b_traj_running_ = (traj_start_time <= loop_time_ && loop_time_ < traj_end_time);

  if (b_traj_running_)
  {
    if (!b_traj_start_)
    {
      p_init_ = p_mes_;
      b_traj_start_ = true;
    }
    else
    {
      switch (traj_type_)
      {
      case TrajectoryType::CUBIC: {
        Vec2<T> p_goal = p_init_ + Vec2<T>::Constant(0.2);  // move EE 10 cm

        p_des_ = planner_->cubic(loop_time_, traj_start_time, traj_end_time, p_init_, p_goal,
                                 Vec2<T>::Zero());
        a_des_.setZero();
        break;
      }
      case TrajectoryType::CIRCULAR: {
        const int repeat(3);
        T circle_radius(0.1);
        T circle_freq = repeat / (traj_end_time - traj_start_time);

        p_des_ =
          planner_->circular(loop_time_, traj_start_time, circle_radius, circle_freq, p_init_);
        a_des_ = planner_->circularDDot(loop_time_, traj_start_time, circle_radius, circle_freq);
        break;
      }
      default:
        p_des_ = p_mes_;
        a_des_.setZero();

        break;
      }
    }
  }
  else
  {
    p_des_ = p_mes_;
    a_des_.setZero();
    b_traj_start_ = false;
  }

  //* control law *//
  /* task-space control */
  Vec2<T> Fc = Mo * a_des_ + kp_.cwiseProduct(p_des_ - p_mes_) + kd_.cwiseProduct(-v_mes_);
  tau_des_ = J_t * Fc;

  /* joint-space control */
  tau_des_ += tau_g;

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

  p_mes_(0) = d->sensordata[5];        // y direction
  p_mes_(1) = d->sensordata[6] - 1.5;  // z direction (1.5 is initial height)

  v_mes_(0) = d->sensordata[12];  // y direction
  v_mes_(1) = d->sensordata[13];  // z direction (1.5 is initial height)
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

template class ComputedTorqueController<float>;
template class ComputedTorqueController<double>;