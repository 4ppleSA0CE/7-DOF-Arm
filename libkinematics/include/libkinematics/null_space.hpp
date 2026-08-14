#pragma once
// Redundancy resolution: null-space projector and secondary-objective gradients
// for a kinematically redundant arm. Secondary motion q_dot_ns = N(q) * grad is
// projected so it does not disturb the end-effector task.
#include <Eigen/Core>
#include <vector>

#include "libkinematics/robot.hpp"

namespace klib {

// Bounding sphere (world frame) for link/obstacle distance costs.
struct Sphere {
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  double radius = 0.0;
};

// Null-space projector N = I - J^+ J (n x n). Motions N*v leave J*q_dot unchanged.
Eigen::MatrixXd null_space_projector(const Eigen::MatrixXd& J);

// Gradient of the center-of-range joint-limit cost H(q) = 1/(2n) sum((q-mid)/range)^2.
// Descend (-grad) to steer away from limits.
Eigen::VectorXd gradient_joint_limit_cost(const Eigen::VectorXd& q, const Eigen::VectorXd& q_min,
                                          const Eigen::VectorXd& q_max);

// Gradient of the manipulability index w = sqrt(det(J J^T)). Ascend to improve.
Eigen::VectorXd gradient_manipulability(const Robot& robot, const Eigen::VectorXd& q);

// Gradient of the summed inverse-distance obstacle cost (link bounding spheres at
// each link origin with the given radii, vs. obstacle spheres). Descend to push away.
Eigen::VectorXd gradient_obstacle_distance(const Robot& robot, const Eigen::VectorXd& q,
                                           const std::vector<double>& link_radii,
                                           const std::vector<Sphere>& obstacles);

}  // namespace klib
