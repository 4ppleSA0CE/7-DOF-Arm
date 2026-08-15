// Motion-QUALITY diagnostic. test_reach only checks where the tip ends up;
// this measures HOW it got there -- the thing that makes the arm look like a
// machine instead of a startled animal.
//
// Per target it reports: peak tip speed / accel / jerk, peak joint speed, how
// long any actuator sits saturated, overshoot past the ball, settling time,
// residual motion once settled, phase flips, and tip-velocity sign reversals
// (the direct signature of "bounces around near the target").
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "redundancy_controller.hpp"

using webviz::RedundancyController;

namespace {

// Per-joint actuator torque limits from the MJCF (<motor ctrlrange="-50 50">
// is further clipped by each joint's actuatorfrcrange).
constexpr double kTauLim[7] = {40, 80, 40, 40, 20, 20, 20};
constexpr double kDt = 0.001;  // MJCF timestep

struct Report {
  std::string name;
  double final_err_mm = 0;
  double peak_tip_speed = 0;   // m/s
  double peak_tip_acc = 0;     // m/s^2
  double peak_tip_jerk = 0;    // m/s^3
  double peak_qd = 0;          // rad/s (worst joint)
  int peak_qd_joint = -1;
  double sat_frac = 0;         // fraction of steps with >=1 actuator at its limit
  double overshoot_mm = 0;     // furthest past the ball after first arrival
  double settle_s = -1;        // time to get and stay within 10 mm
  double residual_speed = 0;   // mean tip speed over the last second
  int phase_flips = 0;
  int reversals = 0;           // tip-velocity direction reversals while moving
};

Eigen::Vector3d vec(const std::array<double, 3>& a) { return {a[0], a[1], a[2]}; }

Report run(const char* name, const Eigen::Vector3d& goal, int steps,
           const Eigen::Vector3d* second_goal = nullptr, int switch_at = 0) {
  RedundancyController c;
  c.set_target(goal);
  Report r;
  r.name = name;

  Eigen::Vector3d p_prev = vec(c.get_state().tip), v_prev = Eigen::Vector3d::Zero(),
                  a_prev = Eigen::Vector3d::Zero();
  Eigen::VectorXd q_prev =
      Eigen::Map<const Eigen::VectorXd>(c.get_state().q.data(), c.get_state().q.size());
  std::string phase_prev = c.get_state().phase;
  long sat_steps = 0;
  bool arrived = false;
  double last_far_t = 0.0;          // last time the tip was >10 mm out
  double closest = 1e9;             // closest the tip has come to the ball so far
  std::vector<double> tail_speeds;  // tip speed over the final second

  for (int i = 0; i < steps; ++i) {
    if (second_goal && i == switch_at) c.set_target(*second_goal);
    c.step();
    const auto s = c.get_state();
    const double t = i * kDt;

    const Eigen::Vector3d p = vec(s.tip);
    const Eigen::Vector3d v = (p - p_prev) / kDt;
    const Eigen::Vector3d a = (v - v_prev) / kDt;
    const Eigen::Vector3d j = (a - a_prev) / kDt;
    const Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(s.q.data(), s.q.size());
    const Eigen::VectorXd qd = (q - q_prev) / kDt;

    // Skip the first few steps: the numerical derivatives are not yet primed.
    if (i > 3) {
      r.peak_tip_speed = std::max(r.peak_tip_speed, v.norm());
      r.peak_tip_acc = std::max(r.peak_tip_acc, a.norm());
      r.peak_tip_jerk = std::max(r.peak_tip_jerk, j.norm());
      for (int k = 0; k < qd.size(); ++k)
        if (std::abs(qd(k)) > r.peak_qd) { r.peak_qd = std::abs(qd(k)); r.peak_qd_joint = k; }
      // A reversal: the tip was moving meaningfully and flipped direction.
      if (v.norm() > 0.02 && v_prev.norm() > 0.02 && v.dot(v_prev) < 0) ++r.reversals;
    }

    for (std::size_t k = 0; k < s.tau.size() && k < 7; ++k)
      if (std::abs(s.tau[k]) >= kTauLim[k] - 1e-6) { ++sat_steps; break; }

    const double err = (vec(s.target) - p).norm();
    if (err < 0.010) arrived = true;
    else last_far_t = t;
    if (arrived) {
      // Overshoot is how far back OUT the tip travels after its closest approach.
      // Taking max(err) from the moment of arrival instead reports the error at the
      // arrival threshold -- i.e. the width of the 10 mm band -- for every target,
      // which made a monotone approach look like 10 mm of overshoot.
      closest = std::min(closest, err);
      r.overshoot_mm = std::max(r.overshoot_mm, (err - closest) * 1000.0);
    }
    if (s.phase != phase_prev) { ++r.phase_flips; phase_prev = s.phase; }
    if (t > (steps - 1) * kDt - 1.0) tail_speeds.push_back(v.norm());

    p_prev = p; v_prev = v; a_prev = a; q_prev = q;
  }

  const auto s = c.get_state();
  r.final_err_mm = s.tip_err_mm;
  r.sat_frac = static_cast<double>(sat_steps) / steps;
  r.settle_s = arrived ? last_far_t : -1.0;
  if (!tail_speeds.empty()) {
    double sum = 0;
    for (double x : tail_speeds) sum += x;
    r.residual_speed = sum / tail_speeds.size();
  }
  return r;
}

void header() {
  std::printf(
      "\n%-26s %8s %7s %7s %9s %7s %6s %8s %7s %8s %6s %5s\n",
      "target", "err_mm", "vmax", "amax", "jmax", "qd_max", "sat%", "over_mm",
      "settle", "resid", "flips", "rev");
  std::printf("%s\n", std::string(120, '-').c_str());
}

void print(const Report& r) {
  std::printf("%-26s %8.1f %7.2f %7.1f %9.0f %7.2f %6.1f %8.1f %7.2f %8.4f %6d %5d\n",
              r.name.c_str(), r.final_err_mm, r.peak_tip_speed, r.peak_tip_acc,
              r.peak_tip_jerk, r.peak_qd, r.sat_frac * 100.0, r.overshoot_mm,
              r.settle_s, r.residual_speed, r.phase_flips, r.reversals);
}

}  // namespace

int main() {
  const struct { const char* name; Eigen::Vector3d g; } cases[] = {
      {"front-high [0.50,0,0.55]",   {0.50, 0.00, 0.55}},
      {"front-low  [0.52,-.01,0.34]", {0.52, -0.01, 0.34}},
      {"right      [0.60,0.30,0.30]", {0.60, 0.30, 0.30}},
      {"left-high  [0.30,-.30,0.70]", {0.30, -0.30, 0.70}},
      {"far-front  [0.65,0,0.45]",   {0.65, 0.00, 0.45}},
      {"diag       [0.35,0.35,0.55]", {0.35, 0.35, 0.55}},
      {"mid        [0.50,0.15,0.40]", {0.50, 0.15, 0.40}},
      {"side+      [0.35,0.45,0.45]", {0.35, 0.45, 0.45}},
      {"side-      [0.35,-.45,0.45]", {0.35, -0.45, 0.45}},
      {"behind     [-.40,0.20,0.50]", {-0.40, 0.20, 0.50}},
      {"near-axis  [0.02,0.02,0.90]", {0.02, 0.02, 0.90}},
      {"out-of-reach [1.6,0,0.5]",   {1.60, 0.00, 0.50}},
  };

  header();
  for (const auto& c : cases) print(run(c.name, c.g, 12000));

  // Re-target mid-flight: an industrial arm should blend, not snap.
  const Eigen::Vector3d g2(0.35, -0.40, 0.55);
  print(run("retarget mid-flight", Eigen::Vector3d(0.60, 0.30, 0.30), 12000, &g2, 1500));
  const Eigen::Vector3d g3(0.50, 0.00, 0.55);
  print(run("out->in recovery", Eigen::Vector3d(1.60, 0.00, 0.50), 14000, &g3, 4000));

  std::printf(
      "\nlegend: vmax m/s | amax m/s^2 | jmax m/s^3 | qd_max rad/s | over_mm = worst\n"
      "        distance past the ball after first arrival | resid = mean tip speed in\n"
      "        the final second (should be ~0 for a settled industrial arm)\n");
  return 0;
}
