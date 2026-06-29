#pragma once
// Shared 6-vector aliases. Twist ordering is [omega; v] (angular first, then
// linear) — Lynch & Park convention, binding project-wide. Wrench ordering is
// [m; f] (moment first), the dual pairing for that twist convention.
#include <Eigen/Core>

namespace klib {

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

// Spatial/body velocity, [omega; v].
using Twist = Vector6d;
// Force/moment, [m; f].
using Wrench = Vector6d;

// Accessors that make the [omega; v] split explicit at call sites.
inline Eigen::Vector3d angular(const Twist& V) noexcept { return V.head<3>(); }
inline Eigen::Vector3d linear(const Twist& V) noexcept { return V.tail<3>(); }

}  // namespace klib
