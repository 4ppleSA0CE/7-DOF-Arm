#pragma once
// Forward kinematics via body-form Product of Exponentials:
//   T(q) = M * exp([B_1] q_1) ... exp([B_n] q_n).
// Twist order [omega; v] (Lynch & Park), binding.
#include <Eigen/Core>
#include <vector>

#include "libkinematics/math/se3.hpp"
#include "libkinematics/math/types.hpp"
#include "libkinematics/robot.hpp"

namespace klib {

class ForwardKinematics {
 public:
  explicit ForwardKinematics(Robot robot);

  // Tool pose in the base/world frame.
  SE3 body_pose(const Eigen::VectorXd& q) const;

  // World pose of each jointed link frame (size = dof).
  std::vector<SE3> link_poses(const Eigen::VectorXd& q) const;

  // Body screw axes as a 6 x n matrix (column i = B_i). Used by the Jacobian.
  const Eigen::Matrix<double, 6, Eigen::Dynamic>& body_screw_axes() const noexcept { return b_; }

  const Robot& robot() const noexcept { return robot_; }

 private:
  Robot robot_;
  Eigen::Matrix<double, 6, Eigen::Dynamic> b_;  // body screws, 6 x n
  Eigen::Matrix<double, 6, Eigen::Dynamic> s_;  // space screws S_i = Ad_M B_i, 6 x n
};

}  // namespace klib
