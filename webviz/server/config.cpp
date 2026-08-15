#include "config.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>

#include <yaml-cpp/yaml.h>

#ifndef WEBVIZ_CONFIG_PATH
#define WEBVIZ_CONFIG_PATH ""  // absolute path to webviz/config.yaml, injected by CMake
#endif

namespace webviz {
namespace {

bool readable(const std::string& p) {
  if (p.empty()) return false;
  std::ifstream f(p);
  return f.good();
}

// Assign only if the key exists, so a partial config file overrides just the
// keys it names and everything else keeps its default.
template <typename T>
void get(const YAML::Node& n, const char* key, T& out) {
  if (n && n[key]) out = n[key].as<T>();
}

// Per-joint arrays accept either 7 values or a single scalar applied to all 7.
void get_joints(const YAML::Node& n, const char* key, JointArray& out) {
  if (!n || !n[key]) return;
  const YAML::Node v = n[key];
  if (v.IsScalar()) {
    out.fill(v.as<double>());
  } else if (v.IsSequence() && v.size() == out.size()) {
    for (std::size_t i = 0; i < out.size(); ++i) out[i] = v[i].as<double>();
  } else {
    std::fprintf(stderr, "[config] '%s' must be a scalar or %zu values; ignored\n", key,
                 out.size());
  }
}

// A non-positive or non-finite tuning value is never a meaningful request, and
// several are actively dangerous rather than merely wrong. A zero velocity /
// acceleration / jerk cap livelocks the trajectory outright: `s` never advances,
// so `moving_` never clears, so no later goal is ever accepted. A negative one
// makes std::clamp's lo > hi, which is undefined behaviour. Fall back to the
// built-in default and say so, rather than propagating it into the control law.
void require_positive(const char* key, double& v, double fallback) {
  if (std::isfinite(v) && v > 0.0) return;
  std::fprintf(stderr, "[config] %s must be positive and finite; using %g\n", key, fallback);
  v = fallback;
}

void require_positive(const char* key, JointArray& v, const JointArray& fallback) {
  for (std::size_t i = 0; i < v.size(); ++i) {
    if (std::isfinite(v[i]) && v[i] > 0.0) continue;
    std::fprintf(stderr, "[config] %s[%zu] must be positive and finite; using %g\n", key, i,
                 fallback[i]);
    v[i] = fallback[i];
  }
}

// Zero is meaningful for these (no integral action, no friction feedforward), a
// negative value is not: it flips the sign of the term it scales.
void require_nonnegative(const char* key, double& v, double fallback) {
  if (std::isfinite(v) && v >= 0.0) return;
  std::fprintf(stderr, "[config] %s must be non-negative and finite; using %g\n", key, fallback);
  v = fallback;
}

}  // namespace

Config load_config(const std::string& path, std::string* loaded_from) {
  Config c;
  std::string file = path;
  if (file.empty()) {
    if (const char* env = std::getenv("WEBVIZ_CONFIG"); env && *env) file = env;
  }
  if (!readable(file)) file = WEBVIZ_CONFIG_PATH;
  if (!readable(file)) file = "config.yaml";
  if (!readable(file)) {
    if (loaded_from) *loaded_from = "";
    return c;  // no file anywhere: built-in defaults
  }

  try {
    const YAML::Node root = YAML::LoadFile(file);
    get(root["server"], "port", c.port);

    const YAML::Node m = root["motion"];
    get_joints(m, "joint_vel_max", c.joint_vel_max);
    get_joints(m, "joint_acc_max", c.joint_acc_max);
    get_joints(m, "joint_jerk_max", c.joint_jerk_max);
    get(m, "speed_scale", c.speed_scale);

    const YAML::Node k = root["control"];
    get(k, "bandwidth", c.bandwidth);
    get(k, "damping_ratio", c.damping_ratio);
    get(k, "ki", c.ki);
    get(k, "integral_clamp", c.integral_clamp);
    get(k, "friction_ff", c.friction_ff);
    get(k, "armature", c.armature);

    const YAML::Node i = root["ik"];
    get(i, "tol", c.ik_tol);
    get(i, "max_iters", c.ik_max_iters);
    get(i, "damping", c.ik_damping);
    get(i, "base_face_gain", c.ik_base_face_gain);
    get(i, "base_face_min_radius", c.ik_base_face_min_radius);

    const YAML::Node o = root["obstacle"];
    get(o, "enabled", c.obstacle_enabled);
    get(o, "radius", c.obstacle_radius);
    get(o, "influence", c.obstacle_influence);
    get(o, "gain", c.obstacle_gain);

    const YAML::Node s = root["stall"];
    get(s, "err", c.stall_err);
    get(s, "time", c.stall_time);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "[config] %s: %s -- using defaults\n", file.c_str(), e.what());
    if (loaded_from) *loaded_from = "";
    return Config{};
  }

  const Config d;  // built-in defaults, used as the fallback for any bad value

  require_positive("motion.joint_vel_max", c.joint_vel_max, d.joint_vel_max);
  require_positive("motion.joint_acc_max", c.joint_acc_max, d.joint_acc_max);
  require_positive("motion.joint_jerk_max", c.joint_jerk_max, d.joint_jerk_max);
  require_positive("motion.speed_scale", c.speed_scale, d.speed_scale);
  if (c.speed_scale > 1.0) {
    std::fprintf(stderr, "[config] motion.speed_scale above 1 is not a speedup; clamping\n");
    c.speed_scale = 1.0;
  }

  require_positive("control.bandwidth", c.bandwidth, d.bandwidth);
  require_positive("control.damping_ratio", c.damping_ratio, d.damping_ratio);
  require_nonnegative("control.ki", c.ki, d.ki);
  require_nonnegative("control.integral_clamp", c.integral_clamp, d.integral_clamp);
  require_nonnegative("control.friction_ff", c.friction_ff, d.friction_ff);
  require_nonnegative("control.armature", c.armature, d.armature);

  require_positive("ik.tol", c.ik_tol, d.ik_tol);
  require_positive("ik.damping", c.ik_damping, d.ik_damping);
  if (c.ik_max_iters < 1) {
    std::fprintf(stderr, "[config] ik.max_iters must be at least 1; using %d\n", d.ik_max_iters);
    c.ik_max_iters = d.ik_max_iters;
  }

  require_positive("obstacle.radius", c.obstacle_radius, d.obstacle_radius);
  require_positive("obstacle.influence", c.obstacle_influence, d.obstacle_influence);
  require_positive("stall.time", c.stall_time, d.stall_time);

  if (c.port <= 0 || c.port > 65535) {
    std::fprintf(stderr, "[config] port %d out of range; using %d\n", c.port, d.port);
    c.port = d.port;
  }

  if (loaded_from) *loaded_from = file;
  return c;
}

}  // namespace webviz
