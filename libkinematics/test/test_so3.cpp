#include "libkinematics/math/so3.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <array>
#include <random>

using klib::SO3;

namespace {
std::mt19937 rng(42);
double urand(double lo, double hi) {
  return std::uniform_real_distribution<double>(lo, hi)(rng);
}
Eigen::Vector3d randVec(double lo, double hi) {
  return Eigen::Vector3d(urand(lo, hi), urand(lo, hi), urand(lo, hi));
}
// A rotation vector with norm < pi (so log round-trips uniquely).
Eigen::Vector3d randSmallOmega() {
  Eigen::Vector3d axis = randVec(-1, 1).normalized();
  if (!axis.allFinite()) axis = Eigen::Vector3d::UnitZ();
  return axis * urand(0.0, M_PI - 1e-3);
}
}  // namespace

TEST(SO3, HatVeeRoundTrip) {
  for (int i = 0; i < 100; ++i) {
    Eigen::Vector3d w = randVec(-5, 5);
    EXPECT_TRUE(SO3::vee(SO3::hat(w)).isApprox(w, 1e-12));
  }
}

TEST(SO3, ExpMatchesAngleAxis) {
  for (int i = 0; i < 1000; ++i) {
    Eigen::Vector3d w = randSmallOmega();
    double angle = w.norm();
    Eigen::Vector3d axis = w.normalized();
    Eigen::Matrix3d ref = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
    EXPECT_TRUE(SO3::exp(w).matrix().isApprox(ref, 1e-10));
  }
}

TEST(SO3, ExpLogRoundTrip) {
  for (int i = 0; i < 1000; ++i) {
    Eigen::Vector3d w = randSmallOmega();
    Eigen::Vector3d back = SO3::exp(w).log();
    EXPECT_LT((back - w).norm(), 1e-10) << "w=" << w.transpose();
  }
}

TEST(SO3, InverseCancels) {
  for (int i = 0; i < 1000; ++i) {
    Eigen::Vector3d w = randVec(-M_PI, M_PI);
    Eigen::Matrix3d prod = (SO3::exp(w) * SO3::exp(-w)).matrix();
    EXPECT_TRUE(prod.isApprox(Eigen::Matrix3d::Identity(), 1e-12));
  }
}

TEST(SO3, IdentityLogIsZero) {
  EXPECT_LT(SO3().log().norm(), 1e-12);
}

TEST(SO3, NearPiLog) {
  // theta close to pi: the ill-conditioned branch must still recover the axis.
  const std::array<Eigen::Vector3d, 5> axes = {
      Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 1, 0), Eigen::Vector3d(0, 0, 1),
      Eigen::Vector3d(1, 1, 0).normalized(), Eigen::Vector3d(1, 1, 1).normalized()};
  for (const Eigen::Vector3d& axis : axes) {
    double theta = M_PI - 1e-6;
    Eigen::Vector3d w = axis * theta;
    SO3 R = SO3::exp(w);
    Eigen::Vector3d back = R.log();
    // log may return the antipodal axis (+/-), so compare the rotations.
    EXPECT_TRUE(SO3::exp(back).matrix().isApprox(R.matrix(), 1e-8));
    EXPECT_NEAR(back.norm(), theta, 1e-4);
  }
}

TEST(SO3, NoHeapAllocationOnHotPath) {
  Eigen::Vector3d w = randSmallOmega();
  SO3 a = SO3::exp(w);
  SO3 b = SO3::exp(randSmallOmega());
  Eigen::internal::set_is_malloc_allowed(false);
  volatile double sink = 0;
  for (int i = 0; i < 100; ++i) {
    SO3 c = a * b;
    sink += c.log().sum() + c.Adjoint().sum();
  }
  Eigen::internal::set_is_malloc_allowed(true);
  (void)sink;
  SUCCEED();
}
