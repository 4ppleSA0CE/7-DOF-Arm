#pragma once
// Numerical inverse kinematics. Damped least squares (Levenberg-Marquardt style)
// with adaptive damping that grows near singularities. Twist order [omega; v].
#include <Eigen/Core>
#include <optional>

#include "libkinematics/fk.hpp"
#include "libkinematics/robot.hpp"

namespace klib {

class IkSolver {
 public:
  virtual ~IkSolver() = default;
  // Solve for q reaching target_pose, starting from q_seed. nullopt on failure.
  virtual std::optional<Eigen::VectorXd> solve(const SE3& target_pose,
                                               const Eigen::VectorXd& q_seed) const = 0;
  void set_max_iterations(int n) { max_iterations_ = n; }
  void set_position_tolerance(double t) { position_tol_ = t; }
  void set_orientation_tolerance(double t) { orientation_tol_ = t; }
  // Random-restart budget when the seed lands in a poor basin (TracIK-style).
  void set_max_restarts(int n) { max_restarts_ = n; }

 protected:
  int max_iterations_ = 100;
  int max_restarts_ = 60;
  double position_tol_ = 1e-5;     // m
  double orientation_tol_ = 1e-5;  // rad
};

class DampedLeastSquaresIk : public IkSolver {
 public:
  explicit DampedLeastSquaresIk(Robot robot);
  std::optional<Eigen::VectorXd> solve(const SE3& target_pose,
                                       const Eigen::VectorXd& q_seed) const override;

 private:
  Robot robot_;
  ForwardKinematics fk_;
};

}  // namespace klib
