#include "libkinematics/fk.hpp"

namespace klib {

ForwardKinematics::ForwardKinematics(Robot robot) : robot_(std::move(robot)) {
  const int n = robot_.dof();
  b_.resize(6, n);
  s_.resize(6, n);
  const Matrix6d Ad_M = robot_.home_pose_M.Adjoint();
  for (int i = 0; i < n; ++i) {
    b_.col(i) = robot_.body_screw_axes[i];
    s_.col(i) = Ad_M * robot_.body_screw_axes[i];  // S_i = Ad_M B_i
  }
}

SE3 ForwardKinematics::body_pose(const Eigen::VectorXd& q) const {
  // Body form: T = M * exp([B_1] q_1) ... exp([B_n] q_n).
  SE3 T = robot_.home_pose_M;
  for (int i = 0; i < robot_.dof(); ++i) {
    T = T * SE3::exp(b_.col(i) * q(i));
  }
  return T;
}

std::vector<SE3> ForwardKinematics::link_poses(const Eigen::VectorXd& q) const {
  // Space form: link i (driven by joint i) world pose =
  //   exp([S_1] q_1) ... exp([S_i] q_i) * M_home_i. Joints after i don't move it.
  const int n = robot_.dof();
  std::vector<SE3> out;
  out.reserve(n);
  SE3 P;  // identity
  for (int i = 0; i < n; ++i) {
    P = P * SE3::exp(s_.col(i) * q(i));
    out.push_back(P * robot_.link_home_frames[i]);
  }
  return out;
}

}  // namespace klib
