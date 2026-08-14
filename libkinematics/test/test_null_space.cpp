#include "libkinematics/null_space.hpp"

#include <gtest/gtest.h>

#include "libkinematics/fk.hpp"
#include "libkinematics/jacobian.hpp"
#include "libkinematics/robot.hpp"

using klib::ForwardKinematics;
using klib::Robot;
using klib::Sphere;

namespace {
Robot robot() { return klib::load_robot_yaml(KINOVA_SCREWS_YAML); }

// Resolved-rate step that holds the EE at target while applying a null-space
// secondary velocity. Returns the new q.
Eigen::VectorXd step(const Robot& r, const ForwardKinematics& fk, const Eigen::VectorXd& q,
                     const klib::SE3& target, const Eigen::VectorXd& secondary, double dt) {
  klib::SE3 T = fk.body_pose(q);
  klib::Twist xerr = (T.inverse() * target).log();      // body-frame task error
  Eigen::MatrixXd J = klib::body_jacobian(r, q);
  Eigen::MatrixXd Jinv = J.completeOrthogonalDecomposition().pseudoInverse();
  Eigen::MatrixXd N = klib::null_space_projector(J);
  Eigen::VectorXd qdot = Jinv * (5.0 * xerr) + N * secondary;  // primary hold + secondary
  return q + qdot * dt;
}
}  // namespace

TEST(NullSpace, ProjectorAnnihilatesTaskMotion) {
  Robot r = robot();
  Eigen::VectorXd q(7);
  q << 0.3, -0.5, 0.7, 0.2, -0.9, 0.4, 0.6;
  Eigen::MatrixXd J = klib::body_jacobian(r, q);
  Eigen::MatrixXd N = klib::null_space_projector(J);
  // J N = 0: null-space motion produces no end-effector twist.
  EXPECT_LT((J * N).cwiseAbs().maxCoeff(), 1e-10);
  // N is idempotent.
  EXPECT_TRUE((N * N).isApprox(N, 1e-9));
}

TEST(NullSpace, ZeroSecondaryMatchesPseudoInverseOnly) {
  Robot r = robot();
  ForwardKinematics fk(r);
  Eigen::VectorXd q(7);
  q << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
  klib::SE3 target = fk.body_pose(q);
  Eigen::VectorXd zero = Eigen::VectorXd::Zero(7);
  Eigen::VectorXd a = step(r, fk, q, target, zero, 0.01);
  // Pseudo-inverse-only step (no null-space term at all).
  klib::Twist xerr = (fk.body_pose(q).inverse() * target).log();
  Eigen::MatrixXd J = klib::body_jacobian(r, q);
  Eigen::VectorXd b = q + J.completeOrthogonalDecomposition().pseudoInverse() * (5.0 * xerr) * 0.01;
  EXPECT_LT((a - b).cwiseAbs().maxCoeff(), 1e-12);
}

TEST(NullSpace, JointLimitCostSteersAwayHoldingPose) {
  Robot r = robot();
  ForwardKinematics fk(r);
  Eigen::VectorXd qmin(7), qmax(7);
  for (int i = 0; i < 7; ++i) { qmin(i) = r.joint_limits[i][0]; qmax(i) = r.joint_limits[i][1]; }
  Eigen::VectorXd q(7);
  q << 3.0, 0.2, -2.9, 0.1, 0.0, 0.1, 0.0;  // joints 0,2 near limits
  klib::SE3 target = fk.body_pose(q);
  double cost0 = ((q - 0.5 * (qmin + qmax)).array() / (qmax - qmin).array()).square().sum();
  for (int k = 0; k < 400; ++k) {
    Eigen::VectorXd g = klib::gradient_joint_limit_cost(q, qmin, qmax);
    q = step(r, fk, q, target, -10.0 * g, 0.01);  // descend the cost
  }
  double cost1 = ((q - 0.5 * (qmin + qmax)).array() / (qmax - qmin).array()).square().sum();
  EXPECT_LT(cost1, cost0);  // steered toward center
  klib::Twist e = (fk.body_pose(q).inverse() * target).log();
  EXPECT_LT(e.head<3>().norm(), 1e-3);
  EXPECT_LT(e.tail<3>().norm(), 1e-3);  // EE pose held
}

TEST(NullSpace, ManipulabilityCostIncreasesHoldingPose) {
  Robot r = robot();
  ForwardKinematics fk(r);
  Eigen::VectorXd q(7);
  q << 0.2, 0.6, -0.3, 0.5, 0.1, -0.4, 0.2;
  klib::SE3 target = fk.body_pose(q);
  double w0 = klib::manipulability(r, q);
  for (int k = 0; k < 300; ++k) {
    Eigen::VectorXd g = klib::gradient_manipulability(r, q);
    q = step(r, fk, q, target, 5.0 * g, 0.01);  // ascend manipulability
  }
  EXPECT_GT(klib::manipulability(r, q), w0);
  klib::Twist e = (fk.body_pose(q).inverse() * target).log();
  EXPECT_LT(e.tail<3>().norm(), 1e-3);
}

TEST(NullSpace, ObstacleCostPushesLinksAwayHoldingPose) {
  Robot r = robot();
  ForwardKinematics fk(r);
  Eigen::VectorXd q(7);
  q << 0.2, 0.8, 0.1, 0.6, 0.0, 0.3, 0.0;
  klib::SE3 target = fk.body_pose(q);
  std::vector<double> radii(7, 0.05);
  auto poses = fk.link_poses(q);
  Sphere obs;
  // Positive clearance so the inverse-distance gradient is active (not floored).
  obs.center = poses[3].translation() + Eigen::Vector3d(0.13, 0.0, 0.0);
  obs.radius = 0.03;
  std::vector<Sphere> obstacles{obs};
  auto nearest = [&](const Eigen::VectorXd& qq) {
    double d = 1e9;
    for (auto& p : fk.link_poses(qq)) d = std::min(d, (p.translation() - obs.center).norm());
    return d;
  };
  double d0 = nearest(q);
  for (int k = 0; k < 300; ++k) {
    Eigen::VectorXd g = klib::gradient_obstacle_distance(r, q, radii, obstacles);
    q = step(r, fk, q, target, -3.0 * g, 0.01);  // descend inverse-distance cost
  }
  EXPECT_GT(nearest(q), d0);  // links moved away
  klib::Twist e = (fk.body_pose(q).inverse() * target).log();
  EXPECT_LT(e.tail<3>().norm(), 2e-3);
}
