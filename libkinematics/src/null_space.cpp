#include "libkinematics/null_space.hpp"

#include <algorithm>
#include <cmath>

#include "libkinematics/fk.hpp"
#include "libkinematics/jacobian.hpp"

namespace klib {

namespace {
Eigen::MatrixXd pseudoinverse(const Eigen::MatrixXd& J) {
  return J.completeOrthogonalDecomposition().pseudoInverse();
}

// Summed inverse clearance cost: sum over (link, obstacle) of 1 / clearance,
// clearance = ||c_link - c_obs|| - r_link - r_obs (floored to avoid blow-up).
double obstacle_cost(const ForwardKinematics& fk, const Eigen::VectorXd& q,
                     const std::vector<double>& link_radii, const std::vector<Sphere>& obstacles) {
  const auto poses = fk.link_poses(q);
  constexpr double kFloor = 1e-3;
  double cost = 0.0;
  for (std::size_t i = 0; i < poses.size(); ++i) {
    const Eigen::Vector3d c = poses[i].translation();
    for (const auto& obs : obstacles) {
      const double clearance = (c - obs.center).norm() - link_radii[i] - obs.radius;
      // Inverse-distance repulsion, linearly extended below kFloor.
      //
      // The obvious form, 1/max(clearance, kFloor), makes the cost CONSTANT for every
      // clearance under the floor -- including every penetration depth -- so its
      // gradient is exactly zero precisely where the push-away is needed most. The
      // extension below is the tangent at kFloor, which keeps the cost C1 and
      // strictly decreasing in clearance all the way through contact, so a link that
      // is already inside an obstacle still feels a force driving it out.
      cost += (clearance >= kFloor)
                  ? 1.0 / clearance
                  : 1.0 / kFloor + (kFloor - clearance) / (kFloor * kFloor);
    }
  }
  return cost;
}
}  // namespace

Eigen::MatrixXd null_space_projector(const Eigen::MatrixXd& J) {
  const int n = static_cast<int>(J.cols());
  return Eigen::MatrixXd::Identity(n, n) - pseudoinverse(J) * J;
}

Eigen::VectorXd gradient_joint_limit_cost(const Eigen::VectorXd& q, const Eigen::VectorXd& q_min,
                                          const Eigen::VectorXd& q_max) {
  const int n = static_cast<int>(q.size());
  Eigen::VectorXd g(n);
  for (int i = 0; i < n; ++i) {
    const double range = q_max(i) - q_min(i);
    const double mid = 0.5 * (q_max(i) + q_min(i));
    g(i) = (q(i) - mid) / (range * range * n);  // d/dq of 1/(2n) sum((q-mid)/range)^2
  }
  return g;
}

Eigen::VectorXd gradient_manipulability(const Robot& robot, const Eigen::VectorXd& q) {
  const int n = robot.dof();
  const double h = 1e-6;
  Eigen::VectorXd g(n);
  for (int i = 0; i < n; ++i) {
    Eigen::VectorXd qp = q, qm = q;
    qp(i) += h;
    qm(i) -= h;
    g(i) = (manipulability(robot, qp) - manipulability(robot, qm)) / (2 * h);
  }
  return g;
}

Eigen::VectorXd gradient_obstacle_distance(const Robot& robot, const Eigen::VectorXd& q,
                                           const std::vector<double>& link_radii,
                                           const std::vector<Sphere>& obstacles) {
  ForwardKinematics fk(robot);
  const int n = robot.dof();
  const double h = 1e-6;
  Eigen::VectorXd g(n);
  for (int i = 0; i < n; ++i) {
    Eigen::VectorXd qp = q, qm = q;
    qp(i) += h;
    qm(i) -= h;
    g(i) = (obstacle_cost(fk, qp, link_radii, obstacles) -
            obstacle_cost(fk, qm, link_radii, obstacles)) /
           (2 * h);
  }
  return g;
}

}  // namespace klib
