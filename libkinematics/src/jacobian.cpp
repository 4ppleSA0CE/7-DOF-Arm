#include "libkinematics/jacobian.hpp"

#include <Eigen/SVD>
#include <cmath>

#include "libkinematics/fk.hpp"

namespace klib {

Jacobian body_jacobian(const Robot& robot, const Eigen::VectorXd& q) {
  const int n = robot.dof();
  Jacobian Jb(6, n);
  // J_b,i = Ad_{T^{-1}} B_i with T = exp([B_{i+1}]q_{i+1}) ... exp([B_n]q_n).
  // Sweep right-to-left, accumulating T.
  SE3 T;  // identity
  for (int i = n - 1; i >= 0; --i) {
    Jb.col(i) = T.inverse().Adjoint() * robot.body_screw_axes[i];
    T = SE3::exp(robot.body_screw_axes[i] * q(i)) * T;
  }
  return Jb;
}

Jacobian space_jacobian(const Robot& robot, const Eigen::VectorXd& q) {
  // J_s = Ad_T J_b, T = body_pose(q). Equivalent to the left-accumulated form.
  ForwardKinematics fk(robot);
  return fk.body_pose(q).Adjoint() * body_jacobian(robot, q);
}

double manipulability(const Robot& robot, const Eigen::VectorXd& q) {
  const Jacobian J = body_jacobian(robot, q);
  const double d = (J * J.transpose()).determinant();
  return std::sqrt(std::max(0.0, d));  // clamp tiny negatives from roundoff
}

bool is_near_singular(const Robot& robot, const Eigen::VectorXd& q, double threshold) {
  const Jacobian J = body_jacobian(robot, q);
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(J);
  return svd.singularValues().minCoeff() < threshold;
}

}  // namespace klib
