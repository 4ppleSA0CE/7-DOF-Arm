#pragma once
// Rigid-body dynamics via the Product-of-Exponentials Recursive Newton-Euler
// Algorithm (Lynch & Park §8.3). Twist order [omega; v] (binding).
// Requires a Robot augmented with dynamics fields (load_dynamics_yaml).
#include <Eigen/Core>

#include "libkinematics/math/types.hpp"
#include "libkinematics/robot.hpp"

namespace klib {

// Full inverse dynamics: tau = M(q) qddot + C(q,qdot) qdot + g(q) + J_b^T f_ext.
//
// SIGN: f_ext is the wrench the TOOL APPLIES TO THE ENVIRONMENT, expressed in the
// tool frame, [m; f] ordering -- the Lynch & Park F_tip convention, which is what
// the backward recursion is seeded with. The joint torques therefore gain
// +J_b^T f_ext, not minus. Feeding in a measured CONTACT wrench (what the
// environment applies to the tool) without negating it doubles the term instead of
// cancelling it, which pushes a contact controller further into the contact.
Eigen::VectorXd inverse_dynamics(const Robot& robot, const Eigen::VectorXd& q,
                                 const Eigen::VectorXd& qdot, const Eigen::VectorXd& qddot,
                                 const Wrench& f_ext = Wrench::Zero());

// Joint-space mass matrix M(q) (n x n, symmetric positive-definite).
Eigen::MatrixXd mass_matrix(const Robot& robot, const Eigen::VectorXd& q);

// Gravity torque g(q).
Eigen::VectorXd gravity_term(const Robot& robot, const Eigen::VectorXd& q);

// Coriolis/centrifugal torque C(q,qdot) qdot.
Eigen::VectorXd coriolis_term(const Robot& robot, const Eigen::VectorXd& q,
                              const Eigen::VectorXd& qdot);

// Full Coriolis matrix C(q,qdot) via Christoffel symbols (so that
// d/dt M(q) - 2 C(q,qdot) is skew-symmetric).
Eigen::MatrixXd coriolis_matrix(const Robot& robot, const Eigen::VectorXd& q,
                                const Eigen::VectorXd& qdot);

}  // namespace klib
