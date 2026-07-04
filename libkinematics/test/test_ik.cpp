#include "libkinematics/ik.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <random>
#include <vector>

#include "libkinematics/fk.hpp"
#include "libkinematics/robot.hpp"

using klib::DampedLeastSquaresIk;
using klib::ForwardKinematics;
using klib::Robot;

namespace {
Eigen::VectorXd randInLimits(const Robot& r, std::mt19937& rng) {
  Eigen::VectorXd q(r.dof());
  for (int i = 0; i < r.dof(); ++i) {
    std::uniform_real_distribution<double> d(r.joint_limits[i][0], r.joint_limits[i][1]);
    q(i) = d(rng);
  }
  return q;
}
}  // namespace

TEST(Ik, SuccessRateAndTimingOnReachablePoses) {
  Robot robot = klib::load_robot_yaml(KINOVA_SCREWS_YAML);
  ForwardKinematics fk(robot);
  DampedLeastSquaresIk solver(robot);
  std::mt19937 rng(2024);

  const int N = 3000;
  int success = 0;
  std::vector<double> times_ms;
  times_ms.reserve(N);

  for (int t = 0; t < N; ++t) {
    Eigen::VectorXd q_target = randInLimits(robot, rng);
    klib::SE3 target = fk.body_pose(q_target);
    Eigen::VectorXd seed = randInLimits(robot, rng);

    auto t0 = std::chrono::high_resolution_clock::now();
    auto sol = solver.solve(target, seed);
    auto t1 = std::chrono::high_resolution_clock::now();
    times_ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());

    if (sol) {
      // Verify the returned solution actually reaches the target and respects limits.
      klib::SE3 reached = fk.body_pose(*sol);
      klib::Twist e = (reached.inverse() * target).log();
      EXPECT_LT(e.head<3>().norm(), 1e-4);
      EXPECT_LT(e.tail<3>().norm(), 1e-4);
      bool in_limits = true;
      for (int i = 0; i < robot.dof(); ++i)
        if ((*sol)(i) < robot.joint_limits[i][0] - 1e-9 ||
            (*sol)(i) > robot.joint_limits[i][1] + 1e-9)
          in_limits = false;
      EXPECT_TRUE(in_limits);
      ++success;
    }
  }

  std::sort(times_ms.begin(), times_ms.end());
  double median_ms = times_ms[times_ms.size() / 2];
  double rate = static_cast<double>(success) / N;
  std::cout << "IK success rate=" << rate * 100 << "%  median=" << median_ms << " ms\n";

  EXPECT_GE(rate, 0.95) << "success rate below 95%";
  EXPECT_LT(median_ms, 5.0) << "median solve time exceeds 5 ms";
}

TEST(Ik, ZeroErrorWhenSeedIsAnswer) {
  Robot robot = klib::load_robot_yaml(KINOVA_SCREWS_YAML);
  ForwardKinematics fk(robot);
  DampedLeastSquaresIk solver(robot);
  std::mt19937 rng(1);
  Eigen::VectorXd q = randInLimits(robot, rng);
  auto sol = solver.solve(fk.body_pose(q), q);
  ASSERT_TRUE(sol.has_value());
  klib::Twist e = (fk.body_pose(*sol).inverse() * fk.body_pose(q)).log();
  EXPECT_LT(e.norm(), 1e-6);
}
