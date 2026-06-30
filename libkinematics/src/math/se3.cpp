#include "libkinematics/math/se3.hpp"

#include <cmath>

namespace klib {

namespace {
constexpr double kSmall = 1e-8;

// G(theta) maps the linear part of a twist through a screw motion:
//   p = G(theta) v,  G = I + (1-cos)/t^2 [w] + (t-sin)/t^3 [w]^2  (w unit-scaled).
Eigen::Matrix3d leftJacobian(const Eigen::Vector3d& omega) noexcept {
  const double t = omega.norm();
  const Eigen::Matrix3d W = SO3::hat(omega);
  if (t < kSmall) {
    return Eigen::Matrix3d::Identity() + 0.5 * W;  // Taylor lead terms.
  }
  const double a = (1.0 - std::cos(t)) / (t * t);
  const double b = (t - std::sin(t)) / (t * t * t);
  return Eigen::Matrix3d::Identity() + a * W + b * W * W;
}
}  // namespace

Eigen::Matrix4d SE3::hat(const Twist& xi) noexcept {
  Eigen::Matrix4d M = Eigen::Matrix4d::Zero();
  M.topLeftCorner<3, 3>() = SO3::hat(xi.head<3>());
  M.topRightCorner<3, 1>() = xi.tail<3>();
  return M;
}

Twist SE3::vee(const Eigen::Matrix4d& M) noexcept {
  Twist xi;
  xi.head<3>() = SO3::vee(M.topLeftCorner<3, 3>());
  xi.tail<3>() = M.topRightCorner<3, 1>();
  return xi;
}

SE3 SE3::exp(const Twist& xi) noexcept {
  const Eigen::Vector3d omega = xi.head<3>();
  const Eigen::Vector3d v = xi.tail<3>();
  const SO3 R = SO3::exp(omega);
  const Eigen::Vector3d p = leftJacobian(omega) * v;
  return SE3(R, p);
}

Twist SE3::log() const noexcept {
  const Eigen::Vector3d omega = rot_.log();
  // v = G(omega)^{-1} p. Invert the 3x3 directly (cheap, no allocation).
  const Eigen::Vector3d v = leftJacobian(omega).inverse() * p_;
  Twist xi;
  xi.head<3>() = omega;
  xi.tail<3>() = v;
  return xi;
}

Matrix6d SE3::Adjoint() const noexcept {
  const Eigen::Matrix3d R = rot_.matrix();
  Matrix6d Ad = Matrix6d::Zero();
  Ad.topLeftCorner<3, 3>() = R;
  Ad.bottomRightCorner<3, 3>() = R;
  Ad.bottomLeftCorner<3, 3>() = SO3::hat(p_) * R;
  return Ad;
}

Eigen::Matrix4d SE3::matrix() const noexcept {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.topLeftCorner<3, 3>() = rot_.matrix();
  T.topRightCorner<3, 1>() = p_;
  return T;
}

}  // namespace klib
