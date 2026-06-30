#include "libkinematics/math/so3.hpp"

#include <algorithm>
#include <cmath>

namespace klib {

namespace {
constexpr double kSmall = 1e-8;  // below this angle, use Taylor expansions.
}

Eigen::Matrix3d SO3::hat(const Eigen::Vector3d& w) noexcept {
  Eigen::Matrix3d m;
  m << 0.0, -w.z(), w.y(),
       w.z(), 0.0, -w.x(),
       -w.y(), w.x(), 0.0;
  return m;
}

Eigen::Vector3d SO3::vee(const Eigen::Matrix3d& M) noexcept {
  return Eigen::Vector3d(M(2, 1), M(0, 2), M(1, 0));
}

SO3 SO3::exp(const Eigen::Vector3d& omega) noexcept {
  const double theta = omega.norm();
  const Eigen::Matrix3d W = hat(omega);
  if (theta < kSmall) {
    // R = I + W + W^2/2 + ... ; Taylor avoids division by theta.
    return SO3(Eigen::Matrix3d::Identity() + W + 0.5 * W * W);
  }
  // Rodrigues: R = I + (sin t / t) W + ((1 - cos t)/t^2) W^2.
  const double a = std::sin(theta) / theta;
  const double b = (1.0 - std::cos(theta)) / (theta * theta);
  return SO3(Eigen::Matrix3d::Identity() + a * W + b * W * W);
}

Eigen::Vector3d SO3::log() const noexcept {
  const double cos_theta = std::clamp((r_.trace() - 1.0) * 0.5, -1.0, 1.0);
  const double theta = std::acos(cos_theta);

  if (theta < kSmall) {
    // Near identity: log(R) ~ vee(R - R^T)/2 (the antisymmetric part).
    return vee(r_ - r_.transpose()) * 0.5;
  }
  if (M_PI - theta > 1e-2) {
    // Generic: vee(R - R^T) = 2 sin(theta) * axis. Well-conditioned away from pi.
    return vee(r_ - r_.transpose()) * (theta / (2.0 * std::sin(theta)));
  }
  // Near pi: sin(theta) ~ 0, so the antisymmetric part is tiny and noisy.
  // Recover the axis from the symmetric part instead. With unit axis w:
  //   S := (R + R^T)/2 = cos(theta) I + (1 - cos(theta)) w w^T
  // => w_i^2 = (S_ii - cos)/(1 - cos),  w_i w_j = S_ij/(1 - cos).
  const Eigen::Matrix3d S = 0.5 * (r_ + r_.transpose());
  const double denom = 1.0 - cos_theta;  // > 0 here
  Eigen::Vector3d w2 = ((S.diagonal().array() - cos_theta) / denom).max(0.0);
  Eigen::Vector3d w = w2.cwiseSqrt();
  // Pivot on the largest component for sign stability; derive the rest from the
  // off-diagonals so all relative signs are consistent.
  int k;
  w.maxCoeff(&k);
  for (int j = 0; j < 3; ++j) {
    if (j != k) w(j) = S(k, j) / (denom * w(k));
  }
  // Fix the global sign from the (small but signed) antisymmetric part.
  const Eigen::Vector3d skew = vee(r_ - r_.transpose());
  if (skew(k) < 0) w = -w;
  return w.normalized() * theta;
}

}  // namespace klib
