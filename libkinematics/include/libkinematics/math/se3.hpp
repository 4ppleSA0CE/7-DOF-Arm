#pragma once
// SE(3): rigid-body transforms as a Lie group. exp/log over twists with the
// [omega; v] ordering (Lynch & Park, binding). Adjoint is 6x6 with that same
// ordering. No heap allocation on the hot path.
#include <Eigen/Core>

#include "libkinematics/math/so3.hpp"
#include "libkinematics/math/types.hpp"

namespace klib {

class SE3 {
 public:
  SE3() noexcept : rot_(), p_(Eigen::Vector3d::Zero()) {}
  SE3(const SO3& R, const Eigen::Vector3d& p) noexcept : rot_(R), p_(p) {}
  explicit SE3(const Eigen::Matrix4d& T) noexcept
      : rot_(T.topLeftCorner<3, 3>()), p_(T.topRightCorner<3, 1>()) {}

  // Exponential map: twist [omega; v] -> SE3 (full screw motion).
  static SE3 exp(const Twist& xi) noexcept;
  // Logarithm: SE3 -> twist [omega; v].
  Twist log() const noexcept;

  // Adjoint, [omega; v] ordering: Ad_T = [[R, 0], [ [p]x R, R ]].
  Matrix6d Adjoint() const noexcept;

  // hat: twist -> 4x4 se(3) matrix; vee is the inverse.
  static Eigen::Matrix4d hat(const Twist& xi) noexcept;
  static Twist vee(const Eigen::Matrix4d& M) noexcept;

  SE3 operator*(const SE3& rhs) const noexcept { return SE3(rot_ * rhs.rot_, rot_ * rhs.p_ + p_); }
  Eigen::Vector3d operator*(const Eigen::Vector3d& p) const noexcept { return rot_ * p + p_; }
  SE3 inverse() const noexcept { return SE3(rot_.inverse(), -(rot_.inverse() * p_)); }

  Eigen::Matrix4d matrix() const noexcept;
  const SO3& rotation() const noexcept { return rot_; }
  const Eigen::Vector3d& translation() const noexcept { return p_; }

 private:
  SO3 rot_;
  Eigen::Vector3d p_;
};

}  // namespace klib
