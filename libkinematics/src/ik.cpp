#include "libkinematics/ik.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <random>

#include "libkinematics/jacobian.hpp"

namespace klib {

namespace {
// One DLS descent from a single seed. Returns q on convergence, else nullopt.
std::optional<Eigen::VectorXd> attempt(const Robot& robot, const ForwardKinematics& fk,
                                       const SE3& target, const Eigen::VectorXd& seed,
                                       int max_iters, double pos_tol, double ori_tol) {
  const int n = robot.dof();
  Eigen::VectorXd q = seed;

  // Adaptive damping: small base, boosted as manipulability w drops near a
  // singularity (Wampler/Nakamura). Step length capped so DLS can't overshoot.
  constexpr double kBaseLambda = 1e-3;
  constexpr double kMaxLambda = 0.1;
  constexpr double kWThresh = 1e-3;
  constexpr double kMaxStep = 0.4;  // rad per iteration, per the dq norm

  for (int iter = 0; iter < max_iters; ++iter) {
    const SE3 T_cur = fk.body_pose(q);
    const Twist V_err = (T_cur.inverse() * target).log();  // body-frame error twist
    if (V_err.head<3>().norm() < ori_tol && V_err.tail<3>().norm() < pos_tol) return q;

    const Jacobian J = body_jacobian(robot, q);
    const double w = std::sqrt(std::max(0.0, (J * J.transpose()).determinant()));
    double lambda2 = kBaseLambda * kBaseLambda;
    if (w < kWThresh) {
      const double s = 1.0 - w / kWThresh;
      lambda2 += kMaxLambda * kMaxLambda * s * s;
    }

    const Eigen::Matrix<double, 6, 6> A =
        J * J.transpose() + lambda2 * Eigen::Matrix<double, 6, 6>::Identity();
    Eigen::VectorXd dq = J.transpose() * A.ldlt().solve(V_err);
    const double dq_norm = dq.norm();
    if (dq_norm > kMaxStep) dq *= kMaxStep / dq_norm;  // damp large steps
    q += dq;
    for (int i = 0; i < n; ++i)
      q(i) = std::clamp(q(i), robot.joint_limits[i][0], robot.joint_limits[i][1]);
  }

  const Twist V_err = (fk.body_pose(q).inverse() * target).log();
  if (V_err.head<3>().norm() < ori_tol && V_err.tail<3>().norm() < pos_tol) return q;
  return std::nullopt;
}
}  // namespace

DampedLeastSquaresIk::DampedLeastSquaresIk(Robot robot) : robot_(robot), fk_(std::move(robot)) {}

std::optional<Eigen::VectorXd> DampedLeastSquaresIk::solve(const SE3& target,
                                                           const Eigen::VectorXd& q_seed) const {
  // Try the provided seed first; on failure, random-restart from configs drawn
  // uniformly within joint limits (deterministic RNG seeded from the target).
  if (auto sol = attempt(robot_, fk_, target, q_seed, max_iterations_, position_tol_,
                         orientation_tol_)) {
    return sol;
  }

  std::size_t h = 0;
  const Twist key = target.log();
  for (int i = 0; i < 6; ++i) h ^= std::hash<double>{}(key(i)) + 0x9e3779b9 + (h << 6) + (h >> 2);
  std::mt19937 rng(static_cast<std::uint32_t>(h));
  const int n = robot_.dof();

  for (int r = 0; r < max_restarts_; ++r) {
    Eigen::VectorXd seed(n);
    for (int i = 0; i < n; ++i) {
      std::uniform_real_distribution<double> d(robot_.joint_limits[i][0], robot_.joint_limits[i][1]);
      seed(i) = d(rng);
    }
    if (auto sol = attempt(robot_, fk_, target, seed, max_iterations_, position_tol_,
                           orientation_tol_)) {
      return sol;
    }
  }
  return std::nullopt;
}

}  // namespace klib
