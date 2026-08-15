// Acceptance/regression battery for the base-rotation reach controller.
// Bare checks (no framework), non-zero exit on failure for ctest. Built by
// CMake so the MuJoCo dylib install-name fixup is handled automatically.
#include <array>
#include <cmath>
#include <cstdio>

#include <Eigen/Dense>

#include "redundancy_controller.hpp"

using webviz::RedundancyController;

static int failures = 0;
static void check(bool ok, const char* msg) {
  std::printf("%s %s\n", ok ? "  ok  " : "FAIL >", msg);
  if (!ok) ++failures;
}

// Home config (matches kQHome in the controller) used as a cold IK seed.
static Eigen::VectorXd home() {
  Eigen::VectorXd q(7);
  q << 0.0, 1.6, 0.0, 1.1, 0.0, 1.2, 0.0;
  return q;
}

// Test 1 (the #1 trap): the position IK must return a base-rotated posture.
static void test_ik_faces_target() {
  RedundancyController c;
  for (double sy : {0.45, -0.45}) {
    const Eigen::Vector3d goal(0.35, sy, 0.45);
    const double azimuth = std::atan2(goal.y(), goal.x());
    auto sol = c.solve_reach_posture(goal, home());
    char buf[128];
    std::snprintf(buf, sizeof(buf), "IK converges for [0.35,%.2f,0.45]", sy);
    check(sol.has_value(), buf);
    if (sol) {
      // Assert on where the ARM POINTS, not on the joint value. joint_1 turns about
      // world -Z, so q1 == azimuth actually means the arm faces the MIRROR of the
      // target -- the old form of this check certified exactly that bug.
      double arm_az = 0.0;
      const bool ok = c.arm_azimuth(*sol, arm_az);
      std::snprintf(buf, sizeof(buf), "  arm points at %.3f, target azimuth %.3f (q1=%.3f)",
                    arm_az, azimuth, (*sol)(0));
      check(ok && std::abs(std::remainder(arm_az - azimuth, 2 * M_PI)) < 0.2, buf);
    }
  }
}

static double dist3(const std::array<double, 3>& a, const std::array<double, 3>& b) {
  double s = 0.0;
  for (int i = 0; i < 3; ++i) { const double d = a[i] - b[i]; s += d * d; }
  return std::sqrt(s);
}

// Test 2: driven end-to-end, the base yaws to face side targets.
static void test_base_rotates() {
  for (double sy : {0.45, -0.45}) {
    RedundancyController c;
    const Eigen::Vector3d goal(0.35, sy, 0.45);
    c.set_target(goal);
    for (int i = 0; i < 8000; ++i) c.step();
    const auto s = c.get_state();
    const double azimuth = std::atan2(goal.y(), goal.x());
    // Driven end-to-end, check the elbow's world azimuth -- i.e. that the arm really
    // is turned toward the target, not that a joint happens to hold a matching number.
    const double elbow_az = std::atan2(s.elbow[1], s.elbow[0]);
    char buf[160];
    std::snprintf(buf, sizeof(buf), "arm points at %.3f, target azimuth %.3f (target y=%.2f)",
                  elbow_az, azimuth, sy);
    check(std::abs(std::remainder(elbow_az - azimuth, 2 * M_PI)) < 0.25, buf);
    std::snprintf(buf, sizeof(buf), "  tip_err=%.1f mm < 20 (target y=%.2f)", s.tip_err_mm, sy);
    check(s.tip_err_mm < 20.0, buf);
  }
}

// Test 3: a spread of reachable targets each converge and settle onto the ball.
static void test_reaches() {
  const Eigen::Vector3d targets[] = {
      {0.50, 0.00, 0.55}, {0.52, -0.01, 0.34}, {0.60, 0.30, 0.30},
      {0.30, -0.30, 0.70}, {0.65, 0.00, 0.45}, {0.35, 0.35, 0.55},
      {0.50, 0.15, 0.40},  {0.45, 0.20, 0.45}, {0.45, -0.20, 0.45}};
  for (const auto& g : targets) {
    RedundancyController c;
    c.set_target(g);
    for (int i = 0; i < 8000; ++i) c.step();
    const auto s = c.get_state();
    char buf[160];
    std::snprintf(buf, sizeof(buf), "reach [%.2f,%.2f,%.2f] tip_err=%.1f mm < 30",
                  g.x(), g.y(), g.z(), s.tip_err_mm);
    check(s.tip_err_mm < 30.0, buf);
  }
}

// Test 4: out of reach -> the tip holds calm (no vibration), no runaway.
static void test_out_of_reach_calm() {
  RedundancyController c;
  c.set_target(Eigen::Vector3d(1.6, 0.0, 0.5));  // far beyond the workspace
  for (int i = 0; i < 6000; ++i) c.step();       // settle at the boundary
  const auto before = c.get_state().tip;
  for (int i = 0; i < 1000; ++i) c.step();       // 1 s
  const auto after = c.get_state().tip;
  const double speed = dist3(before, after) / 1.0;  // m/s over 1 s
  char buf[128];
  std::snprintf(buf, sizeof(buf), "out-of-reach tip speed=%.3f m/s < 0.05 (calm)", speed);
  check(speed < 0.05, buf);
}

// Test 5: far (OOR) then a reachable point -> recovers to the ball.
static void test_out_then_in() {
  RedundancyController c;
  c.set_target(Eigen::Vector3d(1.6, 0.0, 0.5));
  for (int i = 0; i < 3000; ++i) c.step();
  c.set_target(Eigen::Vector3d(0.50, 0.00, 0.55));
  for (int i = 0; i < 8000; ++i) c.step();
  const auto s = c.get_state();
  char buf[128];
  std::snprintf(buf, sizeof(buf), "out->in recovers tip_err=%.1f mm < 30", s.tip_err_mm);
  check(s.tip_err_mm < 30.0, buf);
}

// Test 6: disabling the obstacle leaves the reach undisturbed.
static void test_obstacle_toggle() {
  RedundancyController c;
  c.set_obstacle_enabled(false);
  c.set_target(Eigen::Vector3d(0.50, 0.00, 0.55));
  for (int i = 0; i < 8000; ++i) c.step();
  const auto s = c.get_state();
  char buf[128];
  std::snprintf(buf, sizeof(buf), "obstacle-off reach tip_err=%.1f mm < 30", s.tip_err_mm);
  check(s.tip_err_mm < 30.0 && !s.obstacle_on, buf);
}

// Test 7: pushing the obstacle into the arm makes it reconfigure away while the tool
// stays on the ball. Avoidance is resolved inside the IK, so this is a PLANNED
// posture change, not a reactive shove.
//
// The obstacle is placed just OUTSIDE the settled elbow rather than exactly on it.
// Dead-centre is a degenerate input -- the escape direction is undefined and the
// measured dodge is ~7 mm -- and it is not what dragging the ball into the arm looks
// like. Note the dodge is inherently DIRECTIONAL: with the tool pinned, the null space
// affords moving the elbow outward and downward (measured 107-138 mm) but barely
// sideways or inward (3-4 mm). Making every direction work needs a task-priority QP,
// not a projected potential field.
static void test_obstacle_avoidance() {
  const Eigen::Vector3d goal(0.55, 0.10, 0.45);
  // Where the elbow settles when nothing is in the way, then 6 cm outward from it.
  Eigen::Vector3d obs;
  {
    RedundancyController c;
    c.set_target(goal);
    for (int i = 0; i < 6000; ++i) c.step();
    const auto s = c.get_state();
    obs = Eigen::Vector3d(s.elbow[0] + 0.06, s.elbow[1], s.elbow[2]);
  }
  // Now put the obstacle exactly there, with avoidance on and off.
  double dist[2] = {0, 0}, tip_err[2] = {0, 0};
  for (int on = 0; on < 2; ++on) {
    RedundancyController c;
    c.set_obstacle_enabled(on != 0);
    c.set_target(goal);
    for (int i = 0; i < 6000; ++i) c.step();
    c.set_obstacle(obs);
    for (int i = 0; i < 10000; ++i) c.step();
    const auto s = c.get_state();
    dist[on] = (Eigen::Vector3d(s.elbow[0], s.elbow[1], s.elbow[2]) - obs).norm();
    tip_err[on] = s.tip_err_mm;
  }
  char buf[160];
  std::snprintf(buf, sizeof(buf), "avoidance moves the elbow off the obstacle: %.3f m on vs %.3f m off",
                dist[1], dist[0]);
  check(dist[1] > dist[0] + 0.05, buf);  // >= 5 cm of extra clearance
  std::snprintf(buf, sizeof(buf), "  ...without losing the ball (tip_err=%.2f mm < 20)", tip_err[1]);
  check(tip_err[1] < 20.0, buf);
}

// Test 8: joints 1/3/5/7 are CONTINUOUS on a j2s7s300 (the MJCF says jnt_limited=0;
// the screws YAML's +/-pi describes one turn, not a mechanical stop). A retarget that
// straddles the -x axis must wrap through pi rather than unwind the long way round.
// Before this was handled, a 20 degree retarget cost 340 degrees of base travel.
static void test_continuous_base_wraps() {
  RedundancyController c;
  const Eigen::Vector3d a(-0.45, 0.08, 0.45);   // azimuth ~ +170 deg
  const Eigen::Vector3d b(-0.45, -0.08, 0.45);  // azimuth ~ -170 deg
  c.set_target(a);
  for (int i = 0; i < 9000; ++i) c.step();
  double prev = c.get_state().q[0], travel = 0.0;
  c.set_target(b);
  for (int i = 0; i < 15000; ++i) {
    c.step();
    const double q = c.get_state().q[0];
    travel += std::abs(q - prev);
    prev = q;
  }
  const auto s = c.get_state();
  char buf[176];
  std::snprintf(buf, sizeof(buf),
                "base wraps through pi: %.0f deg of travel for a 20 deg retarget (< 60)",
                travel * 180.0 / M_PI);
  check(travel < 60.0 * M_PI / 180.0, buf);
  std::snprintf(buf, sizeof(buf), "  ...and still lands on the ball (%.2f mm < 20)",
                s.tip_err_mm);
  check(s.tip_err_mm < 20.0, buf);
}

int main() {
  test_ik_faces_target();
  test_base_rotates();
  test_reaches();
  test_out_of_reach_calm();
  test_out_then_in();
  test_obstacle_toggle();
  test_obstacle_avoidance();
  test_continuous_base_wraps();
  std::printf(failures ? "\n%d CHECK(S) FAILED\n" : "\nALL CHECKS PASSED\n", failures);
  return failures ? 1 : 0;
}
