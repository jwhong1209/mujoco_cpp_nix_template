#include <iomanip>
#include <iostream>

#include "FrankaResearch3Model.hpp"

int main(int argc, char ** argv)
{
  const std::string urdf_filename =
    "/home/jwhong/git-repos-jwhong/mujoco_nix_template/assets/model/urdf/fr3_franka_hand.urdf";
  const std::string ee_frame = "fr3_hand_tcp";

  FrankaResearch3Model<double> robot(urdf_filename, ee_frame);
  std::cout << "Model initialized successfully" << std::endl;

  Vec7<double> q = Vec7<double>::Random() * M_PI / 4;  // random configuration
  Vec7<double> dq = Vec7<double>::Zero();

  // 모델 업데이트
  robot.updateKinematics(q, dq);

  // 관성 행렬 계산
  Mat7<double> M = robot.inertia();

  // 결과 출력
  std::cout << "\nConfiguration: " << q.transpose() << std::endl;
  std::cout << "Inertia Matrix:\n" << std::fixed << std::setprecision(6) << M << std::endl;

  // 대칭성 검증
  // bool is_symmetric = true;
  // for (int i = 0; i < 7; ++i)
  // {
  //   for (int j = 0; j < 7; ++j)
  //   {
  //     if (std::abs(M(i, j) - M(j, i)) > 1e-10)
  //     {
  //       is_symmetric = false;
  //       break;
  //     }
  //   }
  // }
  // std::cout << "Matrix is " << (is_symmetric ? "symmetric" : "not symmetric") << std::endl;

  // 양의 정부호성 검증
  // Eigen::SelfAdjointEigenSolver<Mat7<double>> solver(M);
  // bool is_positive_definite = true;
  // for (int i = 0; i < 7; ++i)
  // {
  //   if (solver.eigenvalues()(i) <= 0.0)
  //   {
  //     is_positive_definite = false;
  //     break;
  //   }
  // }
  // std::cout << "Matrix is "
  //           << (is_positive_definite ? "positive definite" : "not positive definite") <<
  //           std::endl;
  // std::cout << "Eigenvalues: " << solver.eigenvalues().transpose() << std::endl;

  return 0;
}