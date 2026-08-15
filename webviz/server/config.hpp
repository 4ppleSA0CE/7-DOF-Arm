#pragma once
// Runtime configuration for the sim server and the motion controller, loaded
// from a YAML file so the arm can be re-tuned without a rebuild.
//
// Every field has a working default, so a missing or partial file is fine --
// only the keys present in the file override the defaults. The search order is
// documented on load_config().
#include <array>
#include <string>

namespace webviz {

// 7 per-joint values, indexed joint_1 .. joint_7.
using JointArray = std::array<double, 7>;

struct Config {
  // --- server ---------------------------------------------------------------
  // 8765 is Foxglove's default WebSocket port, so the sim server stays off it.
  int port = 8770;

  // --- motion limits --------------------------------------------------------
  // The trajectory generator enforces all three by construction. Defaults are
  // roughly the real j2s7s300 (~36 deg/s on the major joints, ~48 on the wrist),
  // which is what makes the arm move like a machine instead of a whip.
  JointArray joint_vel_max{{0.60, 0.60, 0.60, 0.60, 0.80, 0.80, 0.80}};   // rad/s
  JointArray joint_acc_max{{1.20, 1.20, 1.20, 1.20, 1.60, 1.60, 1.60}};   // rad/s^2
  JointArray joint_jerk_max{{6.0, 6.0, 6.0, 6.0, 8.0, 8.0, 8.0}};        // rad/s^3
  double speed_scale = 1.0;  // teach-pendant style global override, (0, 1]

  // --- tracking control -----------------------------------------------------
  // Computed torque: the closed-loop joint error obeys e'' + Kd e' + Kp e = 0,
  // so the response is set by a bandwidth and a damping ratio rather than by
  // seven hand-tuned gains. zeta = 1.0 is critically damped: no overshoot.
  double bandwidth = 14.0;         // rad/s
  double damping_ratio = 1.0;      // 1.0 = critically damped
  double ki = 30.0;                // integral gain (acceleration domain)
  double integral_clamp = 0.30;    // anti-windup bound on the integral state (rad*s)
  double friction_ff = 0.9;        // fraction of the plant's Coulomb friction fed forward
  double armature = 0.1;           // matches the MJCF <joint armature>

  // --- inverse kinematics (solved once per goal, not per step) --------------
  double ik_tol = 5e-4;            // position convergence tolerance (m)
  int ik_max_iters = 300;
  double ik_damping = 1e-3;        // damped-least-squares lambda
  double ik_base_face_gain = 0.6;  // null-space pull turning the base toward the target
  double ik_base_face_min_radius = 0.10;  // below this horizontal radius the azimuth is
                                          // ill-conditioned, so the base yaw is held

  // --- obstacle avoidance (resolved in the IK null space, i.e. in the plan) --
  bool obstacle_enabled = true;
  double obstacle_radius = 0.12;
  double obstacle_influence = 0.18;  // clearance below which the arm starts dodging (m)
  double obstacle_gain = 0.8;

  // --- protective stop ------------------------------------------------------
  // If the trajectory has finished but the arm is stuck (blocked by the table or
  // a joint limit) it gives up instead of leaning on the actuators forever.
  double stall_err = 0.03;   // joint error that counts as "not tracking" (rad)
  double stall_time = 1.0;   // how long it must persist before giving up (s)
};

// Load a Config. `path` empty -> search, in order: $WEBVIZ_CONFIG, the
// repo-root webviz/config.yaml baked in at build time, then ./config.yaml.
// A file that does not exist is not an error: the defaults are returned.
// `loaded_from` (optional) receives the path actually used, or "" for defaults.
Config load_config(const std::string& path = "", std::string* loaded_from = nullptr);

}  // namespace webviz
