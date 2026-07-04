#pragma once
// Geometric Jacobians for body-form PoE. Columns are screw axes; twist order
// [omega; v] (Lynch & Park). For the redundant 7-DOF arm J is 6 x 7.
#include <Eigen/Core>

#include "libkinematics/robot.hpp"

namespace klib {

using Jacobian = Eigen::Matrix<double, 6, Eigen::Dynamic>;

// Body Jacobian J_b: V_b = J_b qdot (end-effector twist in the tool frame).
Jacobian body_jacobian(const Robot& robot, const Eigen::VectorXd& q);

// Space Jacobian J_s: V_s = J_s qdot (twist in the base frame). J_s = Ad_T J_b.
Jacobian space_jacobian(const Robot& robot, const Eigen::VectorXd& q);

// Yoshikawa manipulability index w = sqrt(det(J J^T)) of the body Jacobian.
double manipulability(const Robot& robot, const Eigen::VectorXd& q);

// True when the smallest singular value of the body Jacobian < threshold.
bool is_near_singular(const Robot& robot, const Eigen::VectorXd& q, double threshold);

}  // namespace klib
