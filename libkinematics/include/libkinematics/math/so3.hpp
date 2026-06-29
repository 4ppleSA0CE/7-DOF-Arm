#pragma once
// SO(3): 3D rotations as a Lie group. exp/log via Rodrigues with a Taylor
// fallback near theta = 0. No heap allocation on the hot path.
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace klib {

class SO3 {
 public:
  SO3() noexcept : r_(Eigen::Matrix3d::Identity()) {}
  explicit SO3(const Eigen::Matrix3d& R) noexcept : r_(R) {}

  // Exponential map: rotation vector omega (axis*angle) -> SO3.
  static SO3 exp(const Eigen::Vector3d& omega) noexcept;
  // Logarithm: SO3 -> rotation vector in [0, pi].
  Eigen::Vector3d log() const noexcept;

  // Adjoint of SO(3) is the rotation matrix itself.
  Eigen::Matrix3d Adjoint() const noexcept { return r_; }

  // Skew-symmetric (hat) and its inverse (vee).
  static Eigen::Matrix3d hat(const Eigen::Vector3d& w) noexcept;
  static Eigen::Vector3d vee(const Eigen::Matrix3d& M) noexcept;

  SO3 operator*(const SO3& rhs) const noexcept { return SO3(r_ * rhs.r_); }
  Eigen::Vector3d operator*(const Eigen::Vector3d& p) const noexcept { return r_ * p; }
  SO3 inverse() const noexcept { return SO3(r_.transpose()); }
  const Eigen::Matrix3d& matrix() const noexcept { return r_; }

 private:
  Eigen::Matrix3d r_;
};

}  // namespace klib
