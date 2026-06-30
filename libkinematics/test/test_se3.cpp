#include "libkinematics/math/se3.hpp"

#include <gtest/gtest.h>

#include <random>

#include "libkinematics/math/so3.hpp"

using klib::SE3;
using klib::SO3;
using klib::Twist;

namespace {
std::mt19937 rng(7);
double urand(double lo, double hi) {
  return std::uniform_real_distribution<double>(lo, hi)(rng);
}
// Random twist whose rotation part has norm < pi (unique log).
Twist randTwist() {
  Eigen::Vector3d axis = Eigen::Vector3d(urand(-1, 1), urand(-1, 1), urand(-1, 1));
  if (axis.norm() < 1e-9) axis = Eigen::Vector3d::UnitZ();
  axis.normalize();
  Twist xi;
  xi.head<3>() = axis * urand(0.0, M_PI - 1e-3);
  xi.tail<3>() = Eigen::Vector3d(urand(-1, 1), urand(-1, 1), urand(-1, 1));
  return xi;
}
SE3 randSE3() { return SE3::exp(randTwist()); }
}  // namespace

TEST(SE3, HatVeeRoundTrip) {
  for (int i = 0; i < 100; ++i) {
    Twist xi = randTwist();
    EXPECT_TRUE(SE3::vee(SE3::hat(xi)).isApprox(xi, 1e-12));
  }
}

TEST(SE3, ExpLogRoundTrip) {
  for (int i = 0; i < 1000; ++i) {
    Twist xi = randTwist();
    Twist back = SE3::exp(xi).log();
    // 1e-9, not 1e-10: V^{-1} is mildly ill-conditioned as theta -> pi, costing
    // a few ulp over the pure-rotation SO3 round-trip.
    EXPECT_LT((back - xi).norm(), 1e-9) << "xi=" << xi.transpose();
  }
}

TEST(SE3, InverseCancels) {
  for (int i = 0; i < 1000; ++i) {
    SE3 T = randSE3();
    Eigen::Matrix4d prod = (T * T.inverse()).matrix();
    EXPECT_TRUE(prod.isApprox(Eigen::Matrix4d::Identity(), 1e-12));
  }
}

TEST(SE3, ExpInverseEqualsExpNegative) {
  for (int i = 0; i < 1000; ++i) {
    Twist xi = randTwist();
    Eigen::Matrix4d a = SE3::exp(xi).inverse().matrix();
    Eigen::Matrix4d b = SE3::exp((-xi).eval()).matrix();
    EXPECT_TRUE(a.isApprox(b, 1e-10));
  }
}

// Ad_T * xi == vee( T * hat(xi) * T^{-1} ) for random T and xi.
TEST(SE3, AdjointMatchesConjugation) {
  for (int i = 0; i < 1000; ++i) {
    SE3 T = randSE3();
    Twist xi = randTwist();
    Twist lhs = T.Adjoint() * xi;
    Eigen::Matrix4d conj = T.matrix() * SE3::hat(xi) * T.inverse().matrix();
    Twist rhs = SE3::vee(conj);
    EXPECT_TRUE(lhs.isApprox(rhs, 1e-9)) << "lhs=" << lhs.transpose() << " rhs=" << rhs.transpose();
  }
}

TEST(SE3, MatrixConstructorRoundTrip) {
  for (int i = 0; i < 100; ++i) {
    SE3 T = randSE3();
    EXPECT_TRUE(SE3(T.matrix()).matrix().isApprox(T.matrix(), 1e-12));
  }
}

TEST(SE3, NoHeapAllocationOnHotPath) {
  SE3 a = randSE3();
  SE3 b = randSE3();
  Twist xi = randTwist();
  Eigen::internal::set_is_malloc_allowed(false);
  volatile double sink = 0;
  for (int i = 0; i < 100; ++i) {
    SE3 c = a * b;
    sink += c.log().sum() + (c.Adjoint() * xi).sum();
  }
  Eigen::internal::set_is_malloc_allowed(true);
  (void)sink;
  SUCCEED();
}
