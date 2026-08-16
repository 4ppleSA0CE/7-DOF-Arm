// GoogleBenchmark timing for the from-scratch FK / Jacobian / IK / dynamics.
//
// Every benchmark draws its configurations from a POOL built before the timed
// loop. Calling the RNG inside the loop -- which this file used to do -- charges
// seven uniform_real_distribution draws plus a VectorXd heap allocation to every
// sample, and for FK, whose whole body is ~440 ns, that is a large and completely
// unreported fraction of the reported time.
#include <benchmark/benchmark.h>

#include <random>
#include <vector>

#include "libkinematics/dynamics.hpp"
#include "libkinematics/fk.hpp"
#include "libkinematics/ik.hpp"
#include "libkinematics/jacobian.hpp"
#include "libkinematics/robot.hpp"

using namespace klib;

namespace {
constexpr std::size_t kPool = 512;

Robot robot() {
  Robot r = load_robot_yaml(KINOVA_SCREWS_YAML);
  return load_dynamics_yaml(r, KINOVA_DYNAMICS_YAML);
}

// Fixed pool of random configurations, allocated once. Note these span the full
// [-pi, pi] cube rather than the robot's joint limits, so the averages include
// near-singular and unreachable postures.
std::vector<Eigen::VectorXd> pool(unsigned seed) {
  std::mt19937 g(seed);
  std::uniform_real_distribution<double> d(-M_PI, M_PI);
  std::vector<Eigen::VectorXd> out;
  out.reserve(kPool);
  for (std::size_t k = 0; k < kPool; ++k) {
    Eigen::VectorXd q(7);
    for (int i = 0; i < 7; ++i) q(i) = d(g);
    out.push_back(std::move(q));
  }
  return out;
}
}  // namespace

static void BM_FK(benchmark::State& s) {
  Robot r = robot();
  ForwardKinematics fk(r);
  const auto qs = pool(1);
  std::size_t i = 0;
  for (auto _ : s) benchmark::DoNotOptimize(fk.body_pose(qs[i++ % kPool]));
}
BENCHMARK(BM_FK);

static void BM_LinkPoses(benchmark::State& s) {
  Robot r = robot();
  ForwardKinematics fk(r);
  const auto qs = pool(2);
  std::size_t i = 0;
  for (auto _ : s) benchmark::DoNotOptimize(fk.link_poses(qs[i++ % kPool]));
}
BENCHMARK(BM_LinkPoses);

static void BM_BodyJacobian(benchmark::State& s) {
  Robot r = robot();
  const auto qs = pool(3);
  std::size_t i = 0;
  for (auto _ : s) benchmark::DoNotOptimize(body_jacobian(r, qs[i++ % kPool]));
}
BENCHMARK(BM_BodyJacobian);

// Roughly twice BM_BodyJacobian: space_jacobian builds its own ForwardKinematics
// (which deep-copies the Robot) on every call, then multiplies by the adjoint.
static void BM_SpaceJacobian(benchmark::State& s) {
  Robot r = robot();
  const auto qs = pool(4);
  std::size_t i = 0;
  for (auto _ : s) benchmark::DoNotOptimize(space_jacobian(r, qs[i++ % kPool]));
}
BENCHMARK(BM_SpaceJacobian);

static void BM_IK(benchmark::State& s) {
  Robot r = robot();
  ForwardKinematics fk(r);
  DampedLeastSquaresIk ik(r);
  const auto qs = pool(5), seeds = pool(6);
  std::vector<SE3> targets;
  targets.reserve(kPool);
  for (const auto& q : qs) targets.push_back(fk.body_pose(q));
  std::size_t i = 0;
  for (auto _ : s) {
    const std::size_t k = i++ % kPool;
    benchmark::DoNotOptimize(ik.solve(targets[k], seeds[k]));
  }
}
BENCHMARK(BM_IK);

static void BM_InverseDynamics(benchmark::State& s) {
  Robot r = robot();
  const auto qs = pool(7), qds = pool(8), qdds = pool(9);
  std::size_t i = 0;
  for (auto _ : s) {
    const std::size_t k = i++ % kPool;
    benchmark::DoNotOptimize(inverse_dynamics(r, qs[k], qds[k], qdds[k]));
  }
}
BENCHMARK(BM_InverseDynamics);

// The expensive one: n RNE passes, one per column.
static void BM_MassMatrix(benchmark::State& s) {
  Robot r = robot();
  const auto qs = pool(10);
  std::size_t i = 0;
  for (auto _ : s) benchmark::DoNotOptimize(mass_matrix(r, qs[i++ % kPool]));
}
BENCHMARK(BM_MassMatrix);

BENCHMARK_MAIN();
