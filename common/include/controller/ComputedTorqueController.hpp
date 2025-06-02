#ifndef COMPUTED_TORQUE_CONTROLLER_HPP_
#define COMPUTED_TORQUE_CONTROLLER_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <mujoco/mujoco.h>

/* Custom Libraries */
#include "FrankaResearch3Model.hpp"
#include "ImpedanceController.hpp"

#include "save_data.hpp"

template <typename T>
class ComputedTorqueController
{
private:
  std::unique_ptr<SaveData<T>> logger_;

  std::mutex logging_mtx_;
  std::thread logging_thread_;
  std::atomic<bool> b_logging_running_{ false };

  std::string root_path_;
  std::unique_ptr<FrankaResearch3Model<T>> robot_;

  int iter_ = 0;
  T loop_time_ = 0.0;
  T loop_time_prev_ = 0.0;

  bool b_traj_start_ = false;
  T traj_time_ = 0.0;

  int traj_type_ = 1;

  int dof_;

  Vec7<T> q_mes_;   // measured joint position
  Vec7<T> dq_mes_;  // measured joint velocity

  Vec3<T> ee_pos_init_;
  Vec3<T> ee_pos_des_, ee_pos_cal_, ee_pos_mes_;     // EE Cartesian position
  Quat<T> ee_quat_des_, ee_quat_cal_, ee_quat_mes_;  // EE Cartesian quaternion
  Vec6<T> ee_vel_des_, ee_vel_cal_, ee_vel_mes_;     // EE Cartesian twist
  Vec6<T> ee_acc_des_, ee_acc_mes_;                  // EE Cartesian acceleration

  Vec7<T> q_null_des_;  // desired Null-space configuration
  Vec7<T> tau_des_;     // desired torque command

  Vec6<T> K_task_;  // desired Cartesian stiffness
  Vec6<T> D_task_;  // desired Cartesian damping
  Vec7<T> K_null_;  // desired Null-space stiffness
  Vec7<T> D_null_;  // desired Null-space damping

  std::unique_ptr<ImpedanceController<double>> imp_;
  Vec6<double> F_imp_;

public:
  ComputedTorqueController();

  ComputedTorqueController(const ComputedTorqueController &) = delete;
  ComputedTorqueController & operator=(const ComputedTorqueController &) = delete;
  ComputedTorqueController(ComputedTorqueController &&) = delete;
  ComputedTorqueController & operator=(ComputedTorqueController &&) = delete;

  static ComputedTorqueController & getInstance()
  {
    static ComputedTorqueController instance;
    return instance;
  }

  static void update(const mjModel * m, mjData * d);
  void updateImpl(const mjModel * m, mjData * d);  // * Implement controller here

  /**
   * @brief Get MuJoCo sensor data in MJCF into local member variables
   */
  void getSensorData(const mjModel * m, mjData * d);

  //* ----- LOGGING --------------------------------------------------------------------------------
  void startLogging();
  void stopLogging();
  void dataLoggingLoop();
};

#endif  // COMPUTED_TORQUE_CONTROLLER_HPP_