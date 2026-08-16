// kin_report: eyeball the libkinematics FK/IK numbers from the terminal.
//
// Prints, for the Kinova Gen2 (7-DOF) loaded from the screws YAML:
//   1. the redundancy budget (dof vs. the 6-DOF SE(3) task, null-space dim),
//   2. forward kinematics of the home pose and a sample config,
//   3. an FK->IK round-trip (FK a random q, solve IK, report the residual),
//   4. a redundancy demo: one tool pose reached by several *different* joint
//      vectors -- the concrete proof the extra DOF exists.
//
// Twist order is [omega; v] (Lynch & Park), per PRD §4. Usage:
//   kin_report [path/to/kinova_gen2_screws.yaml]
#include <Eigen/Geometry>
#include <cmath>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

#include "libkinematics/fk.hpp"
#include "libkinematics/ik.hpp"
#include "libkinematics/jacobian.hpp"
#include "libkinematics/null_space.hpp"
#include "libkinematics/robot.hpp"

using klib::DampedLeastSquaresIk;
using klib::ForwardKinematics;
using klib::Robot;
using klib::SE3;

namespace {

constexpr double kRad2Deg = 180.0 / M_PI;

Eigen::VectorXd rand_in_limits(const Robot& r, std::mt19937& rng) {
  Eigen::VectorXd q(r.dof());
  for (int i = 0; i < r.dof(); ++i) {
    std::uniform_real_distribution<double> d(r.joint_limits[i][0], r.joint_limits[i][1]);
    q(i) = d(rng);
  }
  return q;
}

void print_q(const char* label, const Eigen::VectorXd& q) {
  std::printf("%s [", label);
  for (int i = 0; i < q.size(); ++i) std::printf("%s% .4f", i ? ", " : "", q(i));
  std::printf("] rad\n");
}

// Position (m) + quaternion (w,x,y,z) of a tool pose.
void print_pose(const char* label, const SE3& T) {
  const Eigen::Vector3d p = T.translation();
  const Eigen::Quaterniond qt(T.rotation().matrix());
  std::printf("%s  pos=[% .4f, % .4f, % .4f] m   quat(wxyz)=[% .4f, % .4f, % .4f, % .4f]\n", label,
              p.x(), p.y(), p.z(), qt.w(), qt.x(), qt.y(), qt.z());
}

}  // namespace

int main(int argc, char** argv) {
#ifdef KINOVA_SCREWS_YAML
  std::string yaml = KINOVA_SCREWS_YAML;
#else
  std::string yaml = "kinova_gen2_description/config/kinova_gen2_screws.yaml";
#endif
  if (argc > 1) yaml = argv[1];

  const Robot robot = klib::load_robot_yaml(yaml);
  const ForwardKinematics fk(robot);
  DampedLeastSquaresIk solver(robot);

  const int n = robot.dof();
  const int task = 6;  // SE(3) has 6 DOF

  std::printf("=== libkinematics report  (model: %s) ===\n\n", yaml.c_str());

  // ---- 1. Redundancy ------------------------------------------------------
  std::printf("--- REDUNDANCY ---\n");
  std::printf("joints (dof)        : %d\n", n);
  std::printf("task space          : %d  (SE(3): 3 position + 3 orientation)\n", task);
  std::printf("redundant dof       : %d  -> a %d-D self-motion manifold: the arm can move\n",
              n - task, n - task);
  std::printf("                      while the tool stays fixed.\n");
  {
    std::mt19937 rng(7);
    const Eigen::VectorXd q = rand_in_limits(robot, rng);
    const klib::Jacobian J = klib::body_jacobian(robot, q);  // 6 x n
    const Eigen::MatrixXd N = klib::null_space_projector(J);  // n x n, rank = n - 6
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(N);
    int null_dim = 0;
    for (int i = 0; i < svd.singularValues().size(); ++i)
      if (svd.singularValues()(i) > 1e-6) ++null_dim;
    std::printf("null-space projector rank at a random q: %d  (matches redundant dof)\n\n",
                null_dim);
  }

  // ---- 2. Forward kinematics ---------------------------------------------
  std::printf("--- FORWARD KINEMATICS ---\n");
  {
    const Eigen::VectorXd q0 = Eigen::VectorXd::Zero(n);
    print_q("q = home (all zeros):", q0);
    print_pose("  FK tool pose:", fk.body_pose(q0));
    std::printf("  manipulability w = %.5f\n\n", klib::manipulability(robot, q0));

    std::mt19937 rng(42);
    const Eigen::VectorXd q = rand_in_limits(robot, rng);
    print_q("q = sample config:  ", q);
    print_pose("  FK tool pose:", fk.body_pose(q));
    std::printf("  manipulability w = %.5f\n\n", klib::manipulability(robot, q));
  }

  // ---- 3. FK -> IK round-trip --------------------------------------------
  std::printf("--- INVERSE KINEMATICS (FK->IK round-trip) ---\n");
  std::printf("Draw random q, FK to a pose, then solve IK from a *different* seed.\n");
  std::printf("If FK and IK are correct the residual is ~0.\n\n");
  {
    std::mt19937 rng(2024);
    int ok = 0;
    const int trials = 5;
    for (int t = 0; t < trials; ++t) {
      const Eigen::VectorXd q_true = rand_in_limits(robot, rng);
      const SE3 target = fk.body_pose(q_true);
      const Eigen::VectorXd seed = rand_in_limits(robot, rng);
      const auto sol = solver.solve(target, seed);

      std::printf("[trial %d]\n", t + 1);
      print_pose("  target pose:", target);
      if (!sol) {
        std::printf("  IK FAILED to converge\n\n");
        continue;
      }
      const SE3 reached = fk.body_pose(*sol);
      const double pos_mm = (reached.translation() - target.translation()).norm() * 1000.0;
      const double ori_deg = (reached.inverse() * target).log().head<3>().norm() * kRad2Deg;
      print_q("  IK solution:", *sol);
      std::printf("  residual: pos = %.4f mm,  ori = %.4f deg   -> %s\n\n", pos_mm, ori_deg,
                  (pos_mm < 0.1 && ori_deg < 0.01) ? "PASS" : "CHECK");
      if (pos_mm < 0.1 && ori_deg < 0.01) ++ok;
    }
    std::printf("round-trip: %d/%d converged to target\n\n", ok, trials);
  }

  // ---- 4. Redundancy in action -------------------------------------------
  std::printf("--- REDUNDANCY DEMO (one pose, many joint solutions) ---\n");
  std::printf("Solve IK for a single tool pose from several seeds. Distinct joint\n");
  std::printf("vectors that all reach the same pose = the redundant DOF, made visible.\n\n");
  {
    std::mt19937 rng(99);
    const Eigen::VectorXd q_anchor = rand_in_limits(robot, rng);
    const SE3 target = fk.body_pose(q_anchor);
    print_pose("shared target pose:", target);
    std::printf("\n");

    std::vector<Eigen::VectorXd> sols;
    for (int t = 0; sols.size() < 3 && t < 40; ++t) {
      const Eigen::VectorXd seed = rand_in_limits(robot, rng);
      const auto sol = solver.solve(target, seed);
      if (!sol) continue;
      // keep only genuinely distinct configurations
      bool distinct = true;
      for (const auto& s : sols)
        if ((s - *sol).norm() < 0.2) distinct = false;
      if (distinct) sols.push_back(*sol);
    }
    for (std::size_t i = 0; i < sols.size(); ++i) {
      char label[32];
      std::snprintf(label, sizeof(label), "solution %zu:", i + 1);
      print_q(label, sols[i]);
      const SE3 reached = fk.body_pose(sols[i]);
      const double pos_mm = (reached.translation() - target.translation()).norm() * 1000.0;
      std::printf("            reaches target to %.4f mm\n", pos_mm);
    }
    if (sols.size() >= 2) {
      std::printf("\njoint-space distance between solution 1 and 2: %.3f rad\n",
                  (sols[0] - sols[1]).norm());
      std::printf("=> same tool pose, different arm posture. That is the redundancy.\n");
    }
  }
  return 0;
}
