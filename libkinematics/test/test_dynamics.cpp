#include "libkinematics/dynamics.hpp"

#include <gtest/gtest.h>

#include <Eigen/Eigenvalues>
#include <fstream>
#include <random>
#include <sstream>
#include <vector>

#include "libkinematics/jacobian.hpp"
#include "libkinematics/robot.hpp"

using klib::Robot;

namespace {
Robot loadRobot() {
  Robot r = klib::load_robot_yaml(KINOVA_SCREWS_YAML);
  return klib::load_dynamics_yaml(r, KINOVA_DYNAMICS_YAML);
}

std::vector<std::vector<double>> loadCsv(const std::string& path, int cols) {
  std::ifstream in(path);
  EXPECT_TRUE(in.good()) << "cannot open " << path;
  std::vector<std::vector<double>> rows;
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    std::stringstream ss(line);
    std::vector<double> v;
    std::string tok;
    while (std::getline(ss, tok, ',')) v.push_back(std::stod(tok));
    if (static_cast<int>(v.size()) == cols) rows.push_back(v);
  }
  return rows;
}
Eigen::VectorXd seg(const std::vector<double>& v, int s, int n) {
  Eigen::VectorXd o(n);
  for (int i = 0; i < n; ++i) o(i) = v[s + i];
  return o;
}
}  // namespace

// inverse_dynamics agrees with MuJoCo (RNE oracle) on random (q,qd,qdd).
TEST(Dynamics, InverseDynamicsMatchesOracle) {
  Robot robot = loadRobot();
  auto rows = loadCsv(DYN_REFERENCE_CSV, 35);  // q7 qd7 qdd7 tau7 g7
  ASSERT_GE(rows.size(), 1000u);
  double max_err = 0.0;
  for (const auto& r : rows) {
    Eigen::VectorXd q = seg(r, 0, 7), qd = seg(r, 7, 7), qdd = seg(r, 14, 7), tau = seg(r, 21, 7);
    Eigen::VectorXd mine = klib::inverse_dynamics(robot, q, qd, qdd);
    max_err = std::max(max_err, (mine - tau).cwiseAbs().maxCoeff());
  }
  EXPECT_LT(max_err, 1e-8) << "max RNE torque error vs MuJoCo: " << max_err;
}

TEST(Dynamics, GravityTermMatchesOracle) {
  Robot robot = loadRobot();
  auto rows = loadCsv(DYN_REFERENCE_CSV, 35);
  double max_err = 0.0;
  for (const auto& r : rows) {
    Eigen::VectorXd q = seg(r, 0, 7), g = seg(r, 28, 7);
    max_err = std::max(max_err, (klib::gravity_term(robot, q) - g).cwiseAbs().maxCoeff());
  }
  EXPECT_LT(max_err, 1e-8) << "max gravity error vs MuJoCo: " << max_err;
}

TEST(Dynamics, MassMatrixMatchesOracleAndSpd) {
  Robot robot = loadRobot();
  auto rows = loadCsv(MASS_REFERENCE_CSV, 56);  // q7 + M(49)
  ASSERT_GE(rows.size(), 50u);
  double max_err = 0.0;
  for (const auto& r : rows) {
    Eigen::VectorXd q = seg(r, 0, 7);
    Eigen::MatrixXd Mref(7, 7);
    for (int i = 0; i < 7; ++i)
      for (int j = 0; j < 7; ++j) Mref(i, j) = r[7 + i * 7 + j];
    Eigen::MatrixXd M = klib::mass_matrix(robot, q);
    max_err = std::max(max_err, (M - Mref).cwiseAbs().maxCoeff());
    EXPECT_TRUE(M.isApprox(M.transpose(), 1e-10));  // symmetric
    double min_eig = Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>(M).eigenvalues().minCoeff();
    EXPECT_GT(min_eig, 0.0);  // positive-definite
  }
  EXPECT_LT(max_err, 1e-8) << "max mass-matrix error vs MuJoCo: " << max_err;
}

// C qd from the Christoffel matrix matches the RNE Coriolis term.
TEST(Dynamics, CoriolisMatrixConsistentWithTerm) {
  Robot robot = loadRobot();
  std::mt19937 rng(9);
  std::uniform_real_distribution<double> dq(-M_PI, M_PI), dv(-1, 1);
  for (int t = 0; t < 50; ++t) {
    Eigen::VectorXd q(7), qd(7);
    for (int i = 0; i < 7; ++i) { q(i) = dq(rng); qd(i) = dv(rng); }
    Eigen::VectorXd cm = klib::coriolis_matrix(robot, q, qd) * qd;
    Eigen::VectorXd ct = klib::coriolis_term(robot, q, qd);
    EXPECT_LT((cm - ct).cwiseAbs().maxCoeff(), 1e-5);
  }
}

// The external-wrench argument, which nothing exercised before -- which is exactly
// how the header comment drifted to the opposite sign. f_ext is the wrench the TOOL
// APPLIES TO THE ENVIRONMENT (Lynch & Park F_tip), so it contributes +J_b^T f_ext.
TEST(Dynamics, ExternalWrenchAddsBodyJacobianTranspose) {
  Robot robot = loadRobot();
  std::mt19937 rng(31);
  std::uniform_real_distribution<double> dq(-M_PI, M_PI), dw(-8.0, 8.0);
  const Eigen::VectorXd z = Eigen::VectorXd::Zero(7);
  double max_err = 0.0;
  for (int t = 0; t < 50; ++t) {
    Eigen::VectorXd q(7);
    for (int i = 0; i < 7; ++i) q(i) = dq(rng);
    klib::Wrench f;
    for (int i = 0; i < 6; ++i) f(i) = dw(rng);
    const Eigen::VectorXd tau0 = klib::inverse_dynamics(robot, q, z, z);
    const Eigen::VectorXd tauf = klib::inverse_dynamics(robot, q, z, z, f);
    const Eigen::VectorXd expected = klib::body_jacobian(robot, q).transpose() * f;
    max_err = std::max(max_err, (tauf - tau0 - expected).cwiseAbs().maxCoeff());
  }
  EXPECT_LT(max_err, 1e-9) << "tau(f_ext) - tau(0) is not +J_b^T f_ext: " << max_err;
}

// d/dt M(q) - 2 C(q,qd) is skew-symmetric -- the passivity property the impedance
// control law relies on.
//
// NOTE: this is a CONSISTENCY check, not an independent one, and it cannot fail.
// coriolis_matrix builds C from central differences of mass_matrix at h = 1e-6, and
// the Mdot below differences mass_matrix the same way at the same step. Christoffel
// construction then makes C + C^T identically equal to that Mdot for ANY symmetric
// M, so substituting a deliberately wrong mass matrix still leaves |S + S^T| ~ 1e-16.
// It is kept because it documents the property, not because it verifies it. The real
// verification of C is CoriolisMatrixConsistentWithTerm above, which checks the
// Christoffel matrix against the independent Newton-Euler recursion.
TEST(Dynamics, MdotMinusTwoCIsSkew) {
  Robot robot = loadRobot();
  std::mt19937 rng(11);
  std::uniform_real_distribution<double> dq(-M_PI, M_PI), dv(-1, 1);
  const double h = 1e-6;
  for (int t = 0; t < 30; ++t) {
    Eigen::VectorXd q(7), qd(7);
    for (int i = 0; i < 7; ++i) { q(i) = dq(rng); qd(i) = dv(rng); }
    // Mdot = sum_k dM/dq_k * qd_k via central differences.
    Eigen::MatrixXd Mdot = Eigen::MatrixXd::Zero(7, 7);
    for (int k = 0; k < 7; ++k) {
      Eigen::VectorXd qp = q, qm = q;
      qp(k) += h; qm(k) -= h;
      Mdot += (klib::mass_matrix(robot, qp) - klib::mass_matrix(robot, qm)) / (2 * h) * qd(k);
    }
    Eigen::MatrixXd S = Mdot - 2.0 * klib::coriolis_matrix(robot, q, qd);
    EXPECT_LT((S + S.transpose()).cwiseAbs().maxCoeff(), 1e-4) << "Mdot-2C not skew";
  }
}
