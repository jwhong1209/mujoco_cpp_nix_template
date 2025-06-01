#ifndef COMPUTED_TORQUE_CONTROLLER_HPP_
#define COMPUTED_TORQUE_CONTROLLER_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <mujoco/mujoco.h>

#include "DoublePendulumModel.hpp"
#include "TrajectoryGenerator.hpp"
#include "save_data.hpp"

template <typename T>
class ComputedTorqueController
{
private:
  std::unique_ptr<SaveData<T>> logger_;

  std::mutex logging_mtx_;
  std::thread logging_thread_;
  std::atomic<bool> b_logging_running_{ false };

  std::unique_ptr<DoublePendulumModel<T>> robot_;
  std::unique_ptr<TrajectoryGenerator<T>> planner_;

  int iter_ = 0;
  T loop_time_ = 0.0;

  bool b_traj_start_ = false;
  T traj_time_ = 0.0;

  int traj_type_ = 1;
  enum TrajectoryType
  {
    CUBIC = 0,
    CIRCULAR,
  };

  int dof_;

  Vec2<T> q_des_, q_mes_;    // desired / measured joint position
  Vec2<T> dq_des_, dq_mes_;  // desired / measured joint velocity

  Vec2<T> p_init_;
  Vec2<T> p_des_, p_cal_, p_mes_;  // desired / calculated / measured EE Cartesian position
  Vec2<T> v_des_, v_cal_, v_mes_;  // desired / calculated / measured EE Cartesian velocity
  Vec2<T> a_des_;                  // desired EE Cartesian acceleration

  Vec2<T> F_ext_local_;  // external force in local frame
  Vec2<T> F_ext_world_;  // external force in world frame

  Vec2<T> tau_des_;  // desired torque command

  Vec2<T> kp_;  // proportional gain
  Vec2<T> kd_;  // derviative gain

public:
  ComputedTorqueController();

  ComputedTorqueController(const ComputedTorqueController &) = delete;
  ComputedTorqueController & operator=(const ComputedTorqueController &) = delete;
  ComputedTorqueController(ComputedTorqueController &&) = delete;
  ComputedTorqueController & operator=(ComputedTorqueController &&) = delete;

  static ComputedTorqueController & getInstance()
  {
    static ComputedTorqueController controller;
    return controller;
  }

  static void update(const mjModel * m, mjData * d);
  void updateImpl(const mjModel * m, mjData * d);  // * Implement controller here

  /**
   * @brief Get MuJoCo sensor data in MJCF into local member variables
   */
  void getSensorData(const mjModel * m, mjData * d);

  //* ----- LOGGING --------------------------------------------------------------------------------
  // TODO: Implement data logging in separated thread
  void startLogging();
  void stopLogging();
  void dataLoggingLoop();
};

#endif  // COMPUTED_TORQUE_CONTROLLER_HPP_