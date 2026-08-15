#pragma once
// Industrial-style motion controller for the Kinova Gen2. MuJoCo is the physics
// PLANT only (mj_step integrates the equations of motion); every bit of model
// math comes from our own libkinematics (M2-M6). The MJCF, the screws YAML and
// the dynamics YAML are the SAME j2s7s300 model, so FK/J/M/g computed here agree
// with the plant to ~1e-9 (that is exactly what M2/M3/M5 verified).
//
// PLAN, THEN TRACK -- the way a real industrial arm works:
//
//   1. GOAL      The draggable green ball. A new goal (or a moved obstacle)
//                triggers a replan; nothing else does.
//   2. IK        Solved ONCE per replan, not once per control step. Our DLS
//                position IK (M4) resolves the redundancy in its null space
//                (M6): the base yaws to face the target, the arm steers clear of
//                the obstacle, and joints are pushed off their travel limits.
//                The result is a single goal configuration q_goal.
//   3. TRAJECTORY A synchronised, jerk-limited trapezoidal profile carries a
//                reference from the current posture to q_goal. Per-joint
//                velocity / acceleration / jerk limits are enforced BY
//                CONSTRUCTION, so the arm physically cannot snap or whip.
//   4. TRACKING  Computed torque with full feedforward: the joint error obeys
//                e'' + Kd e' + Kp e = 0, critically damped, so the arm follows
//                the plan without overshoot and comes to a complete STOP at the
//                end of it.
//
// There is ONE control law throughout. The previous design switched between a
// joint-space PD and a stiff operational-space PID when the tip came within
// 6 cm of the ball, which stepped the commanded torque from ~14 to ~103 N.m in
// a single tick -- that discontinuity is what made the arm lunge and bounce as
// it arrived.
//
// The streamed State also carries a live FK/IK readout so the dashboard proves
// both work every frame.
//
// The tool task is 3-DOF position, so the [omega; v] twist convention only enters
// through the (full 6xN) Jacobians borrowed from libkinematics.
#include <array>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "config.hpp"
#include "libkinematics/fk.hpp"
#include "libkinematics/ik.hpp"
#include "libkinematics/robot.hpp"

#ifndef SCENE_XML_PATH
#define SCENE_XML_PATH ""  // real absolute path injected by CMake
#endif

// Keep <mujoco/mujoco.h> out of this public header.
struct mjModel_;
struct mjData_;
typedef struct mjModel_ mjModel;
typedef struct mjData_ mjData;

namespace webviz {

// Plain snapshot serialized to the browser.
struct State {
  std::vector<double> q;            // 7 joint positions (the plant's truth)
  std::array<double, 3> tip{}, target{}, obstacle{}, elbow{};
  std::string phase;                // "moving" | "holding" | "stalled" | "unreachable"
  bool obstacle_on = true;          // whether obstacle avoidance is enabled
  bool sweep_on = false;            // null-space posture demo enabled
  double tip_err_mm = 0.0;          // tool-to-ball distance
  double clearance_m = 0.0;         // nearest arm-link-to-obstacle distance
  // Keep-out radius the planner actually enforces. Sent so the dashboard can draw
  // it: the red marker is the obstacle GEOM (0.06 m in the MJCF), but the arm is
  // steered out of a larger sphere, and without showing it the arm looks like it is
  // swerving around empty space.
  double obstacle_radius = 0.12;
  double manip = 0.0;               // Yoshikawa manipulability (our Jacobian)
  std::vector<double> tau;          // last commanded torques (post-saturation)

  // motion telemetry
  double progress = 1.0;            // 0..1 along the current planned move
  double tip_speed = 0.0;           // m/s
  double joint_speed_max = 0.0;     // rad/s, worst joint
  double track_err_rad = 0.0;       // ||q_ref - q||
  double speed_scale = 1.0;         // active teach-pendant speed override
  double eta_s = 0.0;               // estimated seconds left in the current move

  // forward-kinematics readout (our FK)
  std::array<double, 4> fk_quat{};  // tool orientation, (w, x, y, z)

  // live inverse-kinematics check
  std::vector<double> q_ik;         // joints our DLS IK recovered for the tool pose
  bool ik_ok = false;
  double ik_pos_mm = 0.0;
  double ik_ori_deg = 0.0;
  double ik_joint_dist = 0.0;       // ||q_ik - q||: different posture, same pose = redundancy
};

class RedundancyController {
 public:
  explicit RedundancyController(const Config& cfg = load_config(),
                                const std::string& scene_xml = SCENE_XML_PATH);
  ~RedundancyController();
  RedundancyController(const RedundancyController&) = delete;
  RedundancyController& operator=(const RedundancyController&) = delete;

  void reset();
  void set_target(const Eigen::Vector3d& p);
  void set_obstacle(const Eigen::Vector3d& p);
  void set_obstacle_enabled(bool on);
  // Null-space redundancy demo: once the arm has arrived, slowly reconfigure the
  // elbow through a chain of planned null-space steps while the tool stays on the
  // ball. Off by default -- an industrial arm holds still when it is done.
  void set_posture_sweep(bool on);
  // Teach-pendant style global speed override in (0, 1].
  void set_speed_scale(double s);
  void step();
  State get_state() const;

  const Config& config() const { return cfg_; }

  // Position IK that also resolves the redundancy: the returned configuration puts
  // the tool at `goal`, yaws the base to face it, steers the proximal links clear
  // of the obstacle, and stays off the joint travel limits. nullopt if it does not
  // converge (that is the out-of-reach signal). Public so the reach posture can be
  // checked in isolation (the #1 trap is q1 ~ 0). Does NOT touch the M4-verified
  // libkinematics IK, which is still used for the dashboard's inversion proof.
  // `path_from` (optional) additionally requires the straight joint-space path from
  // that posture to the solution to be collision-free. Validating only the ENDPOINTS
  // is not enough: a large base swing can have both ends clear while the arm sweeps
  // straight through the tabletop halfway along.
  std::optional<Eigen::VectorXd> solve_reach_posture(
      const Eigen::Vector3d& goal, const Eigen::VectorXd& seed,
      const Eigen::VectorXd* path_from = nullptr) const;

  // Whether the plant's own collision detection reports this posture as penetrating
  // anything. The analytic scenery model used to STEER the IK is a bounding-sphere
  // approximation, far too conservative to accept or reject with, so MuJoCo's real
  // meshes adjudicate. Used to validate a planned posture before committing to it.
  bool posture_collides(const Eigen::VectorXd& q) const;

  // Whether the straight joint-space move from `a` to `b` passes through anything.
  // The trajectory interpolates linearly in joint space, so this is exactly the path
  // the arm will take.
  bool path_collides(const Eigen::VectorXd& a, const Eigen::VectorXd& b) const;

  // Where the arm's upper section actually points, in world coordinates.
  //
  // This is NOT q(0). link_1 carries quat="0 0 1 0" in the MJCF (a 180 deg flip), so
  // joint_1's local +z maps to world -Z -- our own space Jacobian reports its screw
  // axis as [0, 0, -1], and an FK sweep gives arm azimuth = -q(0) - 0.019. Treating
  // the joint value as the heading points the arm at the MIRROR of the target, which
  // is what the reach used to do: it swung the shoulder 53-90 deg to the wrong side
  // and then contorted joints 2-7 back across the robot's own body to place the tip.
  // Measuring the direction instead is self-correcting and needs no hand-tuned offset.
  //
  // False when the elbow is too close to the base axis for an azimuth to mean anything.
  bool arm_azimuth(const Eigen::VectorXd& q, double& az) const;

  // The joint_1 value that makes the arm face `goal_az`. Calibrated from our own FK
  // at construction, so it stays right if the model changes: azimuth(q1) is measured
  // to be yaw_sign_ * q1 + yaw_offset_, and here that sign is NEGATIVE.
  double base_yaw_for(double goal_az) const;


 private:
  enum class Phase { Moving, Holding, Stalled, Unreachable };

  double nearest_clearance(const Eigen::VectorXd& q) const;
  // Joint-space direction that increases clearance from the obstacle. Used INSIDE
  // the IK (so avoidance shapes the planned posture) rather than as a reactive
  // torque that would fight the tracker mid-motion.
  Eigen::VectorXd obstacle_gradient(const Eigen::VectorXd& q) const;
  // Same, for the fixed scenery (table, ground) read out of the MJCF. Without this
  // the planner happily picks a posture that goes straight through the table and
  // the arm then grinds against the contact.
  Eigen::VectorXd environment_gradient(const Eigen::VectorXd& q) const;
  // Joint-space direction pushing joints away from their own travel limits.
  Eigen::VectorXd joint_limit_gradient(const Eigen::VectorXd& q) const;

  void clamp_to_limits(Eigen::VectorXd& q) const;
  // Per-joint displacement from `from` to `to`, taking the SHORT way round on
  // continuous joints. Everything that reasons about a move has to agree on this or
  // the collision check validates a different path than the arm actually takes.
  Eigen::VectorXd joint_delta(const Eigen::VectorXd& from, const Eigen::VectorXd& to) const;
  // One damped-least-squares descent from a single seed. solve_reach_posture()
  // wraps this in a seed sweep, because one descent can walk the forearm down
  // through the table and never climb back out.
  // `use_secondary = false` gives a bare position descent. The redundancy demo needs
  // that: its whole point is to hold a null-space displacement, and the secondary
  // objectives live in the same null space, so they would simply undo it.
  std::optional<Eigen::VectorXd> solve_reach_once(const Eigen::Vector3d& goal,
                                                  const Eigen::VectorXd& seed,
                                                  bool use_secondary = true) const;

  // Re-solve IK for the current goal and start a new trajectory to it. Cheap
  // enough to call on every goal change, far too expensive to call every step.
  void replan();
  // Build a jerk-limited synchronised profile from the current reference to q_goal.
  void plan_to(const Eigen::VectorXd& q_goal);
  // Advance the profile one control period, producing q_ref_/qd_ref_/qdd_ref_.
  void advance_trajectory(double dt);
  // Next posture for the null-space sweep demo: same tool position, different elbow.
  std::optional<Eigen::VectorXd> next_sweep_posture() const;

  // --- our from-scratch model (loaded from the screws + dynamics YAML) -------
  Config cfg_;
  klib::Robot robot_;                 // same j2s7s300 model as the MJCF plant
  klib::ForwardKinematics fk_;        // FK + link poses            (M2)
  klib::DampedLeastSquaresIk ik_;     // IK: the dashboard inversion proof (M4)

  // --- MuJoCo plant ---------------------------------------------------------
  mjModel* m_ = nullptr;
  mjData* d_ = nullptr;
  mjData* probe_ = nullptr;  // scratch state used to collision-check candidate postures
  int nv_ = 0;

  // Fixed collidable scenery (ground plane, table) extracted from the scene at
  // load time, so the planner knows about the world it is planning in.
  struct StaticGeom {
    int type = 0;                                        // mjtGeom
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
    Eigen::Vector3d size = Eigen::Vector3d::Zero();
  };
  std::vector<StaticGeom> scenery_;

  // Strict upper bound on how far the tool can get from the base: the sum of the
  // link offsets. A goal beyond it is unreachable for certain, so the restart search
  // can be skipped rather than burning its whole budget proving it.
  double max_reach_ = 0.0;

  // Measured mapping from joint_1 to the direction the arm faces (see base_yaw_for).
  double yaw_sign_ = -1.0;
  double yaw_offset_ = 0.0;
  // Signed distance from a world point to one piece of scenery (positive = outside)
  // and the direction to push along to increase it. False if the shape is one we do
  // not model.
  static bool scenery_sdf(const StaticGeom& g, const Eigen::Vector3d& p, double& dist,
                          Eigen::Vector3d& out_dir);

  // Plant properties mirrored so the controller's saturation and friction models
  // agree with what MuJoCo actually applies.
  // Joints 1, 3, 5 and 7 of a j2s7s300 are continuous -- they spin without end, and
  // the MJCF says so (jnt_limited = 0). The screws YAML nonetheless declares +/-pi for
  // them, which is a description of one turn, not a mechanical stop. Taking that
  // literally cost 340 degrees of base travel on a 20 degree retarget that straddled
  // the -x axis, because the arm was not allowed to wrap through pi.
  std::vector<char> continuous_;
  Eigen::VectorXd tau_lim_;      // per-joint torque limit (ctrlrange AND actuatorfrcrange)
  Eigen::VectorXd jnt_damping_;  // MJCF <joint damping>
  Eigen::VectorXd jnt_friction_; // MJCF <joint frictionloss>

  // --- trajectory state -----------------------------------------------------
  Eigen::VectorXd q_start_, q_goal_;                 // endpoints of the current move
  Eigen::VectorXd q_ref_, qd_ref_, qdd_ref_;         // the tracked reference
  double s_ = 1.0, sd_ = 0.0, sdd_ = 0.0;            // path parameter and derivatives
  double sd_max_ = 0.0, sdd_max_ = 0.0, sjerk_max_ = 0.0;  // limits mapped onto s
  bool moving_ = false;
  bool braking_ = false;  // running the stop segment that precedes a mid-flight retarget

  // --- tracking state -------------------------------------------------------
  Eigen::VectorXd integ_j_;      // joint integral (conditionally charged: anti-windup)
  Eigen::VectorXd last_tau_;     // post-saturation, for telemetry
  Eigen::VectorXd sat_;          // per joint: 1 if the last command hit its limit
  double stall_timer_ = 0.0;

  // --- goal / demo state ----------------------------------------------------
  Phase phase_ = Phase::Holding;
  bool obstacle_enabled_ = true;
  bool sweep_enabled_ = false;
  int sweep_dir_ = 1;
  bool replan_pending_ = true;
  Eigen::Vector3d goal_, obstacle_, planned_obstacle_;
  Eigen::VectorXd q_ik_;         // last IK posture (also the warm start)
};

}  // namespace webviz
