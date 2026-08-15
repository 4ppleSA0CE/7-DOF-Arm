// Self-check for the redundancy demo: with the posture sweep enabled the tip stays
// pinned (< 1 cm) while the elbow sweeps a large arc (> 2 cm) over 6 s -- the
// textbook null-space demonstration. The sweep is opt-in: by default the arm holds
// still once it arrives, which is what an industrial machine does. Bare checks (no
// framework) so it runs anywhere and returns non-zero on failure for ctest.
#include <array>
#include <cmath>
#include <cstdio>

#include "redundancy_controller.hpp"

using webviz::RedundancyController;

static double dist3(const std::array<double, 3>& a, const std::array<double, 3>& b) {
  double s = 0.0;
  for (int i = 0; i < 3; ++i) {
    const double d = a[i] - b[i];
    s += d * d;
  }
  return std::sqrt(s);
}

int main() {
  RedundancyController c;   // resets holding, with the ball already on the tool
  c.set_posture_sweep(true);
  const auto s0 = c.get_state();
  const auto tip0 = s0.tip;
  const auto elbow0 = s0.elbow;

  double max_tip_drift = 0.0, max_elbow_sweep = 0.0;
  for (int i = 0; i < 6000; ++i) {  // 6 s @ 1 kHz, one half elbow sweep
    c.step();
    const auto s = c.get_state();
    max_tip_drift = std::max(max_tip_drift, dist3(s.tip, tip0));
    max_elbow_sweep = std::max(max_elbow_sweep, dist3(s.elbow, elbow0));
  }

  std::printf("tip drift %.1f mm, elbow sweep %.1f cm\n", max_tip_drift * 1000.0,
              max_elbow_sweep * 100.0);
  if (max_tip_drift >= 0.01) {
    std::fprintf(stderr, "FAIL: tip drifted %.1f mm (>= 10 mm)\n", max_tip_drift * 1000.0);
    return 1;
  }
  if (max_elbow_sweep <= 0.02) {
    std::fprintf(stderr, "FAIL: elbow swept only %.1f cm (<= 2 cm)\n", max_elbow_sweep * 100.0);
    return 1;
  }
  std::printf("OK\n");
  return 0;
}
