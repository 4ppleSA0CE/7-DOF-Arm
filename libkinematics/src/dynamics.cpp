#include "libkinematics/dynamics.hpp"

#include <vector>

#include "libkinematics/math/so3.hpp"

namespace klib {

namespace {
constexpr double kGravity = 9.81;  // m/s^2, world -z (matches the MJCF oracle)

// Lie bracket (spatial cross) matrix ad_V for V = [w; v], [omega; v] ordering:
//   ad_V = [[ [w]x,  0   ],
//           [ [v]x,  [w]x ]].
Matrix6d adjoint_bracket(const Twist& V) {
  Matrix6d ad = Matrix6d::Zero();
  const Eigen::Matrix3d wx = SO3::hat(V.head<3>());
  ad.topLeftCorner<3, 3>() = wx;
  ad.bottomRightCorner<3, 3>() = wx;
  ad.bottomLeftCorner<3, 3>() = SO3::hat(V.tail<3>());
  return ad;
}

// 6x6 spatial inertia in the link frame, [omega; v] ordering. Derived from
// momentum P = G V:  ang = (Ic - m[c][c]) w + m[c] v,  lin = -m[c] w + m v
// (so that lin = m(v + w x c) = m v_com). Symmetric since [c]^T = -[c].
//   G = [[ Icom - m[c]x[c]x,   m[c]x ],
//        [      -m[c]x,         m I  ]].
Matrix6d spatial_inertia(const LinkInertia& li) {
  const Eigen::Matrix3d cx = SO3::hat(li.com);
  Matrix6d G = Matrix6d::Zero();
  G.topLeftCorner<3, 3>() = li.inertia_com - li.mass * cx * cx;
  G.topRightCorner<3, 3>() = li.mass * cx;
  G.bottomLeftCorner<3, 3>() = -li.mass * cx;
  G.bottomRightCorner<3, 3>() = li.mass * Eigen::Matrix3d::Identity();
  return G;
}

// Core RNE. gravity_on toggles the gravitational base acceleration so the same
// routine serves both full inverse dynamics and the (gravity-free) mass matrix.
Eigen::VectorXd rne(const Robot& robot, const Eigen::VectorXd& q, const Eigen::VectorXd& qd,
                    const Eigen::VectorXd& qdd, const Wrench& f_ext, bool gravity_on) {
  const int n = robot.dof();
  std::vector<SE3> T(n);          // T_{i,i-1}
  std::vector<Twist> V(n), Vd(n); // link spatial velocity / acceleration
  std::vector<Matrix6d> G(n);
  for (int i = 0; i < n; ++i) G[i] = spatial_inertia(robot.link_inertias[i]);

  // Base spatial acceleration carries gravity: Vd_0 = [0; -g_world].
  Twist Vd_prev = Twist::Zero();
  if (gravity_on) Vd_prev.tail<3>() = Eigen::Vector3d(0, 0, kGravity);  // -(-g)
  Twist V_prev = Twist::Zero();

  // Forward pass: propagate velocities and accelerations outward.
  for (int i = 0; i < n; ++i) {
    const SE3 M_i = robot.relative_home[i];            // M_{i-1,i}
    T[i] = SE3::exp((-robot.link_screw_axes[i] * q(i)).eval()) * M_i.inverse();  // T_{i,i-1}
    const Matrix6d Ad = T[i].Adjoint();
    V[i] = Ad * V_prev + robot.link_screw_axes[i] * qd(i);
    Vd[i] = Ad * Vd_prev + adjoint_bracket(V[i]) * robot.link_screw_axes[i] * qd(i) +
            robot.link_screw_axes[i] * qdd(i);
    V_prev = V[i];
    Vd_prev = Vd[i];
  }

  // Backward pass: propagate wrenches inward, extract joint torques.
  Eigen::VectorXd tau(n);
  // Frame {n} -> tool transform for the external wrench: T_{tool,n}.
  Wrench F = f_ext;
  SE3 T_next = robot.relative_home[n].inverse();  // M_{n,tool}^{-1} = T_{tool,n}
  for (int i = n - 1; i >= 0; --i) {
    F = T_next.Adjoint().transpose() * F + G[i] * Vd[i] -
        adjoint_bracket(V[i]).transpose() * (G[i] * V[i]);
    tau(i) = F.dot(robot.link_screw_axes[i]);
    T_next = T[i];  // T_{i,i-1} for the next (inner) link
  }
  return tau;
}
}  // namespace

Eigen::VectorXd inverse_dynamics(const Robot& robot, const Eigen::VectorXd& q,
                                 const Eigen::VectorXd& qdot, const Eigen::VectorXd& qddot,
                                 const Wrench& f_ext) {
  return rne(robot, q, qdot, qddot, f_ext, /*gravity_on=*/true);
}

Eigen::VectorXd gravity_term(const Robot& robot, const Eigen::VectorXd& q) {
  const int n = robot.dof();
  return rne(robot, q, Eigen::VectorXd::Zero(n), Eigen::VectorXd::Zero(n), Wrench::Zero(), true);
}

Eigen::VectorXd coriolis_term(const Robot& robot, const Eigen::VectorXd& q,
                              const Eigen::VectorXd& qdot) {
  const int n = robot.dof();
  // C(q,qd) qd = ID(q,qd,0) - g(q).
  return rne(robot, q, qdot, Eigen::VectorXd::Zero(n), Wrench::Zero(), true) - gravity_term(robot, q);
}

Eigen::MatrixXd mass_matrix(const Robot& robot, const Eigen::VectorXd& q) {
  const int n = robot.dof();
  Eigen::MatrixXd M(n, n);
  const Eigen::VectorXd z = Eigen::VectorXd::Zero(n);
  for (int j = 0; j < n; ++j) {
    Eigen::VectorXd ej = Eigen::VectorXd::Zero(n);
    ej(j) = 1.0;
    // Column j = ID with qd=0, qddot=e_j, gravity off.
    M.col(j) = rne(robot, q, z, ej, Wrench::Zero(), /*gravity_on=*/false);
  }
  return 0.5 * (M + M.transpose());  // symmetrize tiny numerical asymmetry
}

Eigen::MatrixXd coriolis_matrix(const Robot& robot, const Eigen::VectorXd& q,
                                const Eigen::VectorXd& qdot) {
  // Christoffel symbols: C_ij = sum_k 1/2 (dM_ij/dq_k + dM_ik/dq_j - dM_jk/dq_i) qd_k.
  // M derivatives by central finite difference. Note it is the Christoffel
  // CONSTRUCTION, not the differencing scheme, that makes Mdot - 2C skew-symmetric:
  // the identity holds for any symmetric dM, so the step size only affects accuracy.
  const int n = robot.dof();
  const double h = 1e-6;
  std::vector<Eigen::MatrixXd> dM(n);
  for (int k = 0; k < n; ++k) {
    Eigen::VectorXd qp = q, qm = q;
    qp(k) += h;
    qm(k) -= h;
    dM[k] = (mass_matrix(robot, qp) - mass_matrix(robot, qm)) / (2 * h);
  }
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(n, n);
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j)
      for (int k = 0; k < n; ++k)
        C(i, j) += 0.5 * (dM[k](i, j) + dM[j](i, k) - dM[i](j, k)) * qdot(k);
  return C;
}

}  // namespace klib
