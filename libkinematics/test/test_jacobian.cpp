#include "libkinematics/jacobian.hpp"

#include <gtest/gtest.h>

#include <random>
#include <vector>

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

// Ad_T for [omega; v] ordering, built by hand from R and p.
//
// Deliberately does NOT call SE3::Adjoint(). space_jacobian is *implemented* as
// `body_pose(q).Adjoint() * body_jacobian(...)`, so comparing it against that same
// expression restates the implementation character-for-character and passes even if
// Adjoint() were the inverse adjoint -- both sides would move together. An
// independent construction is the only thing that actually pins the function.
klib::Matrix6d adjointByHand(const SE3& T) {
  const Eigen::Matrix3d R = T.rotation().matrix();
  const Eigen::Vector3d p = T.translation();
  Eigen::Matrix3d px;
  px << 0.0, -p.z(), p.y(),
        p.z(), 0.0, -p.x(),
        -p.y(), p.x(), 0.0;
  klib::Matrix6d Ad = klib::Matrix6d::Zero();
  Ad.topLeftCorner<3, 3>() = R;
  Ad.bottomLeftCorner<3, 3>() = px * R;
  Ad.bottomRightCorner<3, 3>() = R;
  return Ad;
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

// The space Jacobian, built independently from the left-accumulated screw form:
// the space screws are S_i = Ad_M B_i, and column i is S_i pushed forward by the
// product of the PRECEDING joint exponentials,
//   J_s col i = Ad_{exp([S_1]q_1) ... exp([S_{i-1}]q_{i-1})} S_i.
// This shares no code path with space_jacobian's own Ad_T J_b implementation.
TEST(Jacobian, SpaceJacobianMatchesLeftAccumulatedScrewForm) {
  Robot robot = klib::load_robot_yaml(KINOVA_SCREWS_YAML);
  const int n = robot.dof();
  std::vector<klib::Twist> S(n);
  const klib::Matrix6d AdM = adjointByHand(robot.home_pose_M);
  for (int i = 0; i < n; ++i) S[i] = AdM * robot.body_screw_axes[i];

  double max_err = 0.0;
  for (int t = 0; t < 100; ++t) {
    Eigen::VectorXd q = randQ(n);
    klib::Jacobian expected(6, n);
    SE3 T;  // identity
    for (int i = 0; i < n; ++i) {
      expected.col(i) = adjointByHand(T) * S[i];
      T = T * SE3::exp(S[i] * q(i));
    }
    klib::Jacobian Js = klib::space_jacobian(robot, q);
    max_err = std::max(max_err, (Js - expected).cwiseAbs().maxCoeff());
  }
  EXPECT_LT(max_err, 1e-9) << "max space-Jacobian error vs screw form: " << max_err;
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
