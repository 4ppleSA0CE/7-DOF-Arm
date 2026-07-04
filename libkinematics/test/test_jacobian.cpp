#include "libkinematics/jacobian.hpp"

#include <gtest/gtest.h>

#include <random>

#include "libkinematics/fk.hpp"
#include "libkinematics/robot.hpp"

using klib::body_jacobian;
using klib::ForwardKinematics;
using klib::manipulability;
using klib::Robot;
using klib::SE3;

namespace {
std::mt19937 rng(123);
Eigen::VectorXd randQ(int n) {
  Eigen::VectorXd q(n);
  std::uniform_real_distribution<double> d(-M_PI, M_PI);
  for (int i = 0; i < n; ++i) q(i) = d(rng);
  return q;
}

// 5-point central-difference body Jacobian column i:
//   f(h) = log( T(q)^{-1} T(q + h e_i) );  J_b col i = f'(0).
Eigen::Matrix<double, 6, Eigen::Dynamic> numericalBodyJacobian(const ForwardKinematics& fk,
                                                               const Eigen::VectorXd& q) {
  const int n = q.size();
  const double h = 1e-4;
  Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, n);
  const SE3 Tinv = fk.body_pose(q).inverse();
  for (int i = 0; i < n; ++i) {
    auto col = [&](double dh) {
      Eigen::VectorXd qd = q;
      qd(i) += dh;
      return (Tinv * fk.body_pose(qd)).log();
    };
    J.col(i) = (col(-2 * h) - 8 * col(-h) + 8 * col(h) - col(2 * h)) / (12 * h);
  }
  return J;
}
}  // namespace

TEST(Jacobian, AnalyticMatchesNumerical) {
  Robot robot = klib::load_robot_yaml(KINOVA_SCREWS_YAML);
  ForwardKinematics fk(robot);
  double max_err = 0.0;
  for (int t = 0; t < 1000; ++t) {
    Eigen::VectorXd q = randQ(robot.dof());
    auto Ja = body_jacobian(robot, q);
    auto Jn = numericalBodyJacobian(fk, q);
    max_err = std::max(max_err, (Ja - Jn).cwiseAbs().maxCoeff());
  }
  EXPECT_LT(max_err, 1e-5) << "max body-Jacobian error vs 5-point diff: " << max_err;
}

TEST(Jacobian, SpaceEqualsAdjointTimesBody) {
  Robot robot = klib::load_robot_yaml(KINOVA_SCREWS_YAML);
  ForwardKinematics fk(robot);
  for (int t = 0; t < 100; ++t) {
    Eigen::VectorXd q = randQ(robot.dof());
    // Concrete types, not auto: the product is a lazy Eigen expression whose
    // temporary operands would dangle.
    klib::Jacobian Js = klib::space_jacobian(robot, q);
    klib::Jacobian expected = fk.body_pose(q).Adjoint() * body_jacobian(robot, q);
    EXPECT_TRUE(Js.isApprox(expected, 1e-10));
  }
}

TEST(Jacobian, ManipulabilityMatchesDefinitionAndNonneg) {
  Robot robot = klib::load_robot_yaml(KINOVA_SCREWS_YAML);
  for (int t = 0; t < 100; ++t) {
    Eigen::VectorXd q = randQ(robot.dof());
    auto J = body_jacobian(robot, q);
    double w = manipulability(robot, q);
    EXPECT_GE(w, 0.0);
    double ref = std::sqrt((J * J.transpose()).determinant());
    EXPECT_NEAR(w, ref, 1e-9);
  }
}

TEST(Jacobian, ManipulabilityDropsApproachingSingularity) {
  Robot robot = klib::load_robot_yaml(KINOVA_SCREWS_YAML);
  const int n = robot.dof();
  // Manipulability collapses to ~0 at a singularity. Drive toward one by
  // gradient descent on w (numerical) from a random start; confirm w shrinks
  // by orders of magnitude and that is_near_singular then trips, while a
  // generic config is flagged non-singular.
  std::mt19937 rng2(7);
  Eigen::VectorXd q = randQ(n);
  const double w_start = manipulability(robot, q);
  const double h = 1e-5, step = 0.3;
  for (int it = 0; it < 600; ++it) {
    Eigen::VectorXd g(n);
    for (int i = 0; i < n; ++i) {
      Eigen::VectorXd qp = q, qm = q;
      qp(i) += h; qm(i) -= h;
      g(i) = (manipulability(robot, qp) - manipulability(robot, qm)) / (2 * h);
    }
    q -= step * g;  // descend manipulability toward a singularity
  }
  const double w_sing = manipulability(robot, q);
  EXPECT_LT(w_sing, 0.05 * w_start) << "manipulability did not collapse near singularity";
  EXPECT_TRUE(klib::is_near_singular(robot, q, 1e-2));        // singular config flagged
  EXPECT_FALSE(klib::is_near_singular(robot, randQ(n), 1e-6));  // generic config not flagged
}
