#include "redundancy_controller.hpp"

#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <utility>

#include <mujoco/mujoco.h>

#include "libkinematics/dynamics.hpp"
#include "libkinematics/jacobian.hpp"
#include "libkinematics/null_space.hpp"
#include "libkinematics/math/so3.hpp"

namespace webviz {

namespace {
constexpr double kQHome[7] = {0.0, 1.6, 0.0, 1.1, 0.0, 1.2, 0.0};
constexpr int kElbowLink = 3;      // link_poses index of link_4 (the elbow)
// Which link's world azimuth stands for "the direction the arm faces". link_4 (the
// elbow) is the obvious choice and the wrong one: obstacle avoidance reconfigures the
// elbow, so using it makes base facing and avoidance fight over the same quantity and
// the arm stops dodging. link_3 is the first link far enough off the base axis to have
// a meaningful azimuth, and it is carried almost entirely by joint_1.
constexpr int kYawRefLink = 3;
constexpr int kMaxAvoidLink = 3;   // avoidance acts on base -> elbow only (see below)
constexpr double kLinkRadius = 0.06;  // bounding-sphere radius used for clearance
constexpr double kRad2Deg = 180.0 / M_PI;
constexpr double kReplanGoal = 1e-3;   // goal must move this far (m) to trigger a replan
constexpr double kReplanObst = 1e-2;   // ditto for the obstacle
constexpr double kSweepStep = 0.12;    // rad of null-space travel per sweep segment.
                                       // Small on purpose: the reference is interpolated
                                       // along a joint-space CHORD, which only tracks the
                                       // curved null-space manifold if the step is short.
constexpr double kEnvPad = 0.045;      // arm-link padding used against the scenery
constexpr double kEnvInfluence = 0.10; // clearance below which the planner steers away
constexpr int kMaxEnvLink = 5;         // links checked against the scenery (skip the hand,
                                       // which has to be able to work close to the table)
// How deep a MuJoCo contact has to be before a candidate posture is rejected as
// "inside the furniture". The convex hulls of these meshes graze by a few mm in
// perfectly good postures (measured: 6 mm for a posture that tracks fine), while a
// posture that genuinely puts the forearm through the tabletop reads 26-97 mm across
// several contacts. 15 mm sits cleanly between the two.
constexpr double kPenetrationLimit = 0.015;
constexpr double kClearanceFloor = 0.3;  // residual weight on the clearance terms
constexpr int kIkRestarts = 24;        // random restarts before a goal is called unreachable
constexpr int kSettleIters = 40;       // null-space refinement allowed after the tool arrives
constexpr int kStagnantIters = 30;     // non-improving iterations before a seed is abandoned

// The screws + dynamics YAML describe the same model the MJCF plant integrates.
klib::Robot load_model() {
  klib::Robot r = klib::load_robot_yaml(KINOVA_SCREWS_YAML);
  return klib::load_dynamics_yaml(std::move(r), KINOVA_DYNAMICS_YAML);
}

Eigen::VectorXd read_joints(const double* src, int n) {
  Eigen::VectorXd v(n);
  for (int i = 0; i < n; ++i) v(i) = src[i];
  return v;
}

Eigen::VectorXd to_vec(const JointArray& a, int n) {
  Eigen::VectorXd v(n);
  for (int i = 0; i < n; ++i) v(i) = a[static_cast<std::size_t>(i)];
  return v;
}

// Shortest signed angular difference, so a base-yaw command never takes the long
// way round when the target crosses the -x axis.
double wrap_pi(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}
}  // namespace

RedundancyController::RedundancyController(const Config& cfg, const std::string& scene_xml)
    : cfg_(cfg), robot_(load_model()), fk_(robot_), ik_(robot_) {
  char err[1000] = "";
  m_ = mj_loadXML(scene_xml.c_str(), nullptr, err, sizeof(err));
  if (!m_) throw std::runtime_error(std::string("mj_loadXML failed: ") + err);
  d_ = mj_makeData(m_);
  probe_ = mj_makeData(m_);
  nv_ = m_->nv;
  if (nv_ != robot_.dof())
    throw std::runtime_error("MJCF DOF does not match the libkinematics model");
  ik_.set_max_restarts(4);  // bounds the dashboard readout's worst-case solve time

  // Mirror the plant's actuator and friction properties. The command is clipped to
  // BOTH the actuator's ctrlrange and the joint's actuatorfrcrange -- MuJoCo applies
  // the tighter of the two, and the anti-windup logic below has to agree with it or
  // the integral keeps charging against a limit the controller cannot see.
  continuous_.assign(nv_, 0);
  tau_lim_ = Eigen::VectorXd::Constant(nv_, 1e9);
  jnt_damping_ = Eigen::VectorXd::Zero(nv_);
  jnt_friction_ = Eigen::VectorXd::Zero(nv_);
  for (int i = 0; i < nv_; ++i) {
    jnt_damping_(i) = m_->dof_damping[i];
    jnt_friction_(i) = m_->dof_frictionloss[i];
    const int j = m_->dof_jntid[i];
    if (j >= 0 && !m_->jnt_limited[j]) continuous_[i] = 1;  // spins without end
    if (j >= 0 && m_->jnt_actfrclimited[j])
      tau_lim_(i) = std::min(tau_lim_(i), std::min(-m_->jnt_actfrcrange[2 * j],
                                                   m_->jnt_actfrcrange[2 * j + 1]));
  }
  for (int a = 0; a < m_->nu && a < nv_; ++a)
    if (m_->actuator_ctrllimited[a])
      tau_lim_(a) = std::min(tau_lim_(a), std::min(-m_->actuator_ctrlrange[2 * a],
                                                   m_->actuator_ctrlrange[2 * a + 1]));

  // Learn the fixed scenery from the scene file. A body belongs to the ARM if it
  // has a joint, or is an ancestor or descendant of one; everything else that can
  // collide (the ground plane, the table) is something the planner has to avoid.
  std::vector<char> is_arm(m_->nbody, 0);
  for (int b = 0; b < m_->nbody; ++b)
    if (m_->body_jntnum[b] > 0) is_arm[b] = 1;
  for (int b = 1; b < m_->nbody; ++b)          // bodies are in topological order
    if (is_arm[m_->body_parentid[b]] && m_->body_parentid[b] != 0) is_arm[b] = 1;
  for (int b = m_->nbody - 1; b > 0; --b)      // ...and back up to the roots
    if (is_arm[b]) is_arm[m_->body_parentid[b]] = 1;
  is_arm[0] = 0;                               // the world itself is scenery
  for (int g = 0; g < m_->ngeom; ++g) {
    if (!m_->geom_contype[g] && !m_->geom_conaffinity[g]) continue;  // visual only
    if (is_arm[m_->geom_bodyid[g]]) continue;
    StaticGeom sg;
    sg.type = m_->geom_type[g];
    if (sg.type != mjGEOM_PLANE && sg.type != mjGEOM_BOX && sg.type != mjGEOM_SPHERE) {
      std::fprintf(stderr, "[controller] scenery geom %d has unmodelled type %d; "
                           "the planner will not avoid it\n", g, sg.type);
      continue;
    }
    for (int k = 0; k < 3; ++k) {
      sg.pos(k) = m_->geom_pos[3 * g + k];
      sg.size(k) = m_->geom_size[3 * g + k];
    }
    const Eigen::Quaterniond quat(m_->geom_quat[4 * g], m_->geom_quat[4 * g + 1],
                                  m_->geom_quat[4 * g + 2], m_->geom_quat[4 * g + 3]);
    sg.rot = quat.normalized().toRotationMatrix();
    // geom_pos/quat are body-relative; fold in the (fixed) body pose.
    const int b = m_->geom_bodyid[g];
    Eigen::Vector3d bp(m_->body_pos[3 * b], m_->body_pos[3 * b + 1], m_->body_pos[3 * b + 2]);
    const Eigen::Quaterniond bq(m_->body_quat[4 * b], m_->body_quat[4 * b + 1],
                                m_->body_quat[4 * b + 2], m_->body_quat[4 * b + 3]);
    const Eigen::Matrix3d bR = bq.normalized().toRotationMatrix();
    sg.pos = bp + bR * sg.pos;
    sg.rot = bR * sg.rot;
    scenery_.push_back(sg);
  }

  // Calibrate how joint_1 maps to the direction the arm faces, using our own FK
  // rather than assuming it. On this model link_1 carries quat="0 0 1 0" (a 180 deg
  // flip), so joint_1 turns the arm about world -Z and the azimuth is the NEGATIVE of
  // the joint value plus a small shoulder offset. Assuming q1 == azimuth aims the
  // whole upper arm at the mirror of the target.
  {
    Eigen::VectorXd qa(nv_);
    for (int i = 0; i < nv_ && i < 7; ++i) qa(i) = kQHome[i];
    auto az_at = [&](double q1) {
      qa(0) = q1;
      const Eigen::Vector3d p = fk_.body_pose(qa).translation();
      return std::atan2(p.y(), p.x());
    };
    const double a0 = az_at(0.0), a1 = az_at(0.5);
    yaw_sign_ = (std::remainder(a1 - a0, 2.0 * M_PI) > 0.0) ? 1.0 : -1.0;
    yaw_offset_ = a0;
  }

  // Summing the link offsets gives a strict upper bound on the tool's reach.
  for (const klib::SE3& rel : robot_.relative_home) max_reach_ += rel.translation().norm();

  q_start_ = q_goal_ = q_ref_ = Eigen::VectorXd::Zero(nv_);
  qd_ref_ = qdd_ref_ = Eigen::VectorXd::Zero(nv_);
  integ_j_ = last_tau_ = sat_ = Eigen::VectorXd::Zero(nv_);
  q_ik_ = Eigen::VectorXd::Zero(nv_);
  obstacle_enabled_ = cfg_.obstacle_enabled;
  reset();
}

RedundancyController::~RedundancyController() {
  if (probe_) mj_deleteData(probe_);
  if (d_) mj_deleteData(d_);
  if (m_) mj_deleteModel(m_);
}

bool RedundancyController::posture_collides(const Eigen::VectorXd& q) const {
  for (int i = 0; i < m_->nq; ++i) probe_->qpos[i] = (i < nv_) ? q(i) : d_->qpos[i];
  mj_kinematics(m_, probe_);
  mj_collision(m_, probe_);
  for (int c = 0; c < probe_->ncon; ++c)
    if (probe_->contact[c].dist < -kPenetrationLimit) return true;
  return false;
}

bool RedundancyController::path_collides(const Eigen::VectorXd& a,
                                         const Eigen::VectorXd& b) const {
  // Sample density is a tradeoff: too coarse steps over a thin obstacle, too fine
  // costs a replan its realtime budget. 24 samples puts the worst-case gap on a
  // full-travel joint at a few degrees, well under the tabletop's thickness.
  constexpr int kSamples = 24;
  const Eigen::VectorXd d = joint_delta(a, b);  // the route the planner will take
  for (int i = 1; i < kSamples; ++i) {  // endpoints are checked by the caller
    const double t = static_cast<double>(i) / kSamples;
    if (posture_collides(a + t * d)) return true;
  }
  return false;
}

void RedundancyController::reset() {
  for (int i = 0; i < 7 && i < m_->nq; ++i) d_->qpos[i] = kQHome[i];
  for (int i = 0; i < nv_; ++i) d_->qvel[i] = 0.0;
  mj_forward(m_, d_);  // settle the plant

  const Eigen::VectorXd q = read_joints(d_->qpos, nv_);
  goal_ = fk_.body_pose(q).translation();  // OUR FK; green ball starts on the tool
  obstacle_ = fk_.link_poses(q)[kElbowLink].translation() + Eigen::Vector3d(0.0, 0.55, 0.0);
  planned_obstacle_ = obstacle_;
  q_start_ = q_goal_ = q_ref_ = q_ik_ = q;
  qd_ref_.setZero();
  qdd_ref_.setZero();
  integ_j_.setZero();
  sat_.setZero();
  last_tau_.setZero();
  s_ = 1.0;
  sd_ = sdd_ = 0.0;
  moving_ = false;
  braking_ = false;
  replan_pending_ = false;  // already at the goal
  stall_timer_ = 0.0;
  sweep_dir_ = 1;
  phase_ = Phase::Holding;
}

void RedundancyController::set_target(const Eigen::Vector3d& p) {
  if ((p - goal_).norm() < kReplanGoal) return;
  goal_ = p;
  replan_pending_ = true;
}

void RedundancyController::set_obstacle(const Eigen::Vector3d& p) {
  obstacle_ = p;
  // Only a materially moved obstacle is worth re-planning around.
  if (obstacle_enabled_ && (obstacle_ - planned_obstacle_).norm() > kReplanObst)
    replan_pending_ = true;
}

void RedundancyController::set_obstacle_enabled(bool on) {
  if (on == obstacle_enabled_) return;
  obstacle_enabled_ = on;
  replan_pending_ = true;  // the avoidance term entering/leaving changes the posture
}

void RedundancyController::set_posture_sweep(bool on) {
  sweep_enabled_ = on;
  if (!on) return;
  sweep_dir_ = 1;
}

void RedundancyController::set_speed_scale(double s) {
  cfg_.speed_scale = std::clamp(s, 0.05, 1.0);
}

// Distance from the nearest arm link (bounding sphere) to the obstacle sphere.
double RedundancyController::nearest_clearance(const Eigen::VectorXd& q) const {
  const auto poses = fk_.link_poses(q);
  double best = 1e9;
  for (const auto& pose : poses)
    best = std::min(best, (pose.translation() - obstacle_).norm() - kLinkRadius -
                              cfg_.obstacle_radius);
  return best;
}

// Joint-space direction that increases clearance from the obstacle, built from the
// point Jacobians of the proximal chain. The magnitude ramps with
// (influence - clearance) so it KEEPS GROWING under penetration -- the case an
// inverse-clearance cost floors to zero.
//
// Only the PROXIMAL arm (base -> elbow) is steered: those points have redundant
// freedom to move while the tool stays on the ball. Pushing the wrist/hand would
// just fight the position task, since they are rigid with the tool.
Eigen::VectorXd RedundancyController::obstacle_gradient(const Eigen::VectorXd& q) const {
  const auto poses = fk_.link_poses(q);
  const klib::Jacobian Js = klib::space_jacobian(robot_, q);  // 6 x n space screws
  Eigen::VectorXd g = Eigen::VectorXd::Zero(nv_);

  // Push the world point c (moved by joints 0..upto) away from the obstacle. For a
  // space screw [omega; v], the velocity of a world point c is omega x c + v, so
  // that row is the point Jacobian column.
  auto push = [&](const Eigen::Vector3d& c, int upto) {
    const Eigen::Vector3d d = c - obstacle_;
    const double dist = d.norm();
    const double clearance = dist - kLinkRadius - cfg_.obstacle_radius;
    if (clearance >= cfg_.obstacle_influence) return;
    const Eigen::Vector3d dir =
        dist > 1e-6 ? Eigen::Vector3d(d / dist) : Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d f = (cfg_.obstacle_influence - clearance) * dir;
    for (int j = 0; j <= upto; ++j)
      g(j) += (Js.col(j).head<3>().cross(c) + Js.col(j).tail<3>()).dot(f);
  };

  // Each link origin plus the midpoint of the segment to the next link, so an
  // obstacle sitting mid-forearm (between two joint frames) still registers.
  const int last = std::min<int>(kMaxAvoidLink, static_cast<int>(poses.size()) - 1);
  for (int i = 0; i <= last; ++i) {
    push(poses[i].translation(), i);
    if (i + 1 < static_cast<int>(poses.size()))
      push(0.5 * (poses[i].translation() + poses[i + 1].translation()), i);
  }
  return g;
}

bool RedundancyController::scenery_sdf(const StaticGeom& g, const Eigen::Vector3d& p,
                                       double& dist, Eigen::Vector3d& out_dir) {
  const Eigen::Vector3d local = g.rot.transpose() * (p - g.pos);
  switch (g.type) {
    case mjGEOM_PLANE: {
      // Treated as infinite: a finite ground plane the arm could reach around is not
      // a case this scene has, and being conservative here is free.
      dist = local.z();
      out_dir = g.rot.col(2);
      return true;
    }
    case mjGEOM_SPHERE: {
      const double n = local.norm();
      dist = n - g.size.x();
      out_dir = n > 1e-9 ? Eigen::Vector3d(g.rot * (local / n)) : Eigen::Vector3d::UnitZ();
      return true;
    }
    case mjGEOM_BOX: {
      const Eigen::Vector3d d = local.cwiseAbs() - g.size;
      if ((d.array() > 0.0).any()) {  // outside: distance to the nearest face/edge
        const Eigen::Vector3d outside = d.cwiseMax(0.0);
        dist = outside.norm();
        Eigen::Vector3d dir = outside.cwiseProduct(local.cwiseSign());
        out_dir = g.rot * (dist > 1e-9 ? Eigen::Vector3d(dir / dist) : Eigen::Vector3d::UnitZ());
      } else {                        // inside: leave along the shallowest axis
        int axis = 0;
        d.maxCoeff(&axis);
        dist = d(axis);               // negative
        Eigen::Vector3d dir = Eigen::Vector3d::Zero();
        dir(axis) = local(axis) >= 0.0 ? 1.0 : -1.0;
        out_dir = g.rot * dir;
      }
      return true;
    }
    default:
      return false;
  }
}

// Joint-space direction that lifts the arm off the fixed scenery. Same projected
// point-Jacobian construction as the sphere obstacle, applied to every static geom
// the scene declared. This is what stops the planner from choosing a posture that
// reaches THROUGH the table -- previously the IK found such a posture happily, the
// arm drove into the contact, and the tracker sat there straining against it.
Eigen::VectorXd RedundancyController::environment_gradient(const Eigen::VectorXd& q) const {
  Eigen::VectorXd g = Eigen::VectorXd::Zero(nv_);
  if (scenery_.empty()) return g;
  const auto poses = fk_.link_poses(q);
  const klib::Jacobian Js = klib::space_jacobian(robot_, q);

  auto push = [&](const Eigen::Vector3d& c, int upto) {
    for (const StaticGeom& sg : scenery_) {
      double dist = 0.0;
      Eigen::Vector3d dir;
      if (!scenery_sdf(sg, c, dist, dir)) continue;
      const double clearance = dist - kEnvPad;
      if (clearance >= kEnvInfluence) continue;
      const Eigen::Vector3d f = (kEnvInfluence - clearance) * dir;
      for (int j = 0; j <= upto; ++j)
        g(j) += (Js.col(j).head<3>().cross(c) + Js.col(j).tail<3>()).dot(f);
    }
  };

  const int last = std::min<int>(kMaxEnvLink, static_cast<int>(poses.size()) - 1);
  for (int i = 0; i <= last; ++i) {
    push(poses[i].translation(), i);
    if (i + 1 < static_cast<int>(poses.size()))
      push(0.5 * (poses[i].translation() + poses[i + 1].translation()), i);
  }
  return g;
}

// Repels each joint from its own travel limit, so the planned posture never parks
// against a hard stop where the tracker would grind against the constraint.
Eigen::VectorXd RedundancyController::joint_limit_gradient(const Eigen::VectorXd& q) const {
  Eigen::VectorXd g = Eigen::VectorXd::Zero(nv_);
  for (int i = 0; i < nv_ && i < static_cast<int>(robot_.joint_limits.size()); ++i) {
    if (continuous_[i]) continue;  // nothing to be repelled from
    const double lo = robot_.joint_limits[i][0], hi = robot_.joint_limits[i][1];
    const double span = hi - lo;
    if (span <= 0.0) continue;
    const double margin = 0.15 * span;
    if (q(i) < lo + margin) g(i) = (lo + margin - q(i)) / margin;
    else if (q(i) > hi - margin) g(i) = (hi - margin - q(i)) / margin;
  }
  return g;
}

double RedundancyController::base_yaw_for(double goal_az) const {
  return yaw_sign_ * std::remainder(goal_az - yaw_offset_, 2.0 * M_PI);
}

bool RedundancyController::arm_azimuth(const Eigen::VectorXd& q, double& az) const {
  const Eigen::Vector3d p = fk_.link_poses(q)[kYawRefLink].translation();
  if (p.head<2>().norm() < cfg_.ik_base_face_min_radius) return false;
  az = std::atan2(p.y(), p.x());
  return true;
}

void RedundancyController::clamp_to_limits(Eigen::VectorXd& q) const {
  for (int i = 0; i < nv_ && i < static_cast<int>(robot_.joint_limits.size()); ++i) {
    if (continuous_[i]) continue;  // no mechanical stop to clamp against
    q(i) = std::clamp(q(i), robot_.joint_limits[i][0], robot_.joint_limits[i][1]);
  }
}

Eigen::VectorXd RedundancyController::joint_delta(const Eigen::VectorXd& from,
                                                  const Eigen::VectorXd& to) const {
  Eigen::VectorXd d = to - from;
  for (int i = 0; i < nv_; ++i)
    if (continuous_[i]) d(i) = std::remainder(d(i), 2.0 * M_PI);  // short way round
  return d;
}

// Position-only damped least squares with the redundancy resolved in the null
// space (M6): base facing + obstacle clearance + joint-limit centering. Because
// the secondary terms are projected through (I - J^+ J) they steer the posture
// WITHOUT disturbing the tool position, so the base can face the target without
// trading away reach accuracy.
//
// This runs once per replan, so it can afford a tight tolerance and a real
// iteration budget -- the old design solved it every 1 ms control step, which
// forced a loose 2 cm tolerance and let the solution jitter between steps.
std::optional<Eigen::VectorXd> RedundancyController::solve_reach_once(
    const Eigen::Vector3d& goal, const Eigen::VectorXd& seed, bool use_secondary) const {
  Eigen::VectorXd q = seed;
  clamp_to_limits(q);

  // Face the target. Within ik_base_face_min_radius of the base axis the azimuth is
  // atan2 of two near-zero numbers -- ill-conditioned, and it would spin the base for
  // a millimetre of target motion -- so the current yaw is left alone instead.
  const bool face = goal.head<2>().norm() > cfg_.ik_base_face_min_radius;
  const double goal_az = std::atan2(goal.y(), goal.x());

  const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
  const Eigen::MatrixXd In = Eigen::MatrixXd::Identity(nv_, nv_);
  const double lam2 = cfg_.ik_damping * cfg_.ik_damping;

  // The null-space secondary (base facing, obstacle and scenery clearance,
  // joint-limit centering) is ANNEALED across the iterations. Held at full strength
  // it never terminates -- the clearance terms have no equilibrium when the clearance
  // they ask for is not kinematically achievable, so they keep pushing and the
  // position error never settles. Annealing lets them choose which branch to be in
  // early, then hands the last iterations to the pure position descent.
  double err_norm = 1e9;
  int settling = 0;       // iterations spent refining the null space after the tool arrives
  double best_err = 1e9;  // best position error this seed has managed
  int stagnant = 0;       // consecutive iterations that failed to improve on it
  for (int it = 0; it < cfg_.ik_max_iters; ++it) {
    const klib::SE3 T = fk_.body_pose(q);
    const Eigen::Vector3d err = goal - T.translation();
    err_norm = err.norm();

    // Give up on a seed that has stopped improving. A collapsing step is not a
    // sufficient stopping test on its own: for an out-of-reach goal the tool sits
    // against the workspace boundary with a large residual error, so the DLS step
    // stays big, and the continuous joints (1/3/5/7) can keep rotating indefinitely
    // without ever reducing it. Checked before the Jacobian and the gradients, which
    // are the expensive part of the iteration.
    if (err_norm < best_err - 1e-7) {
      best_err = err_norm;
      stagnant = 0;
    } else if (err_norm >= cfg_.ik_tol && ++stagnant > kStagnantIters) {
      break;
    }

    const Eigen::MatrixXd Jp =
        T.rotation().matrix() * klib::body_jacobian(robot_, q).bottomRows(3);  // 3 x n
    const Eigen::MatrixXd Jinv =
        Jp.transpose() * (Jp * Jp.transpose() + lam2 * I3).inverse();  // n x 3

    // Base facing and joint-limit centering each have a reachable equilibrium (q0 at
    // the azimuth; every joint inside its margin), so they run at full strength all
    // the way through. The two CLEARANCE terms do not -- when the clearance they ask
    // for is not kinematically achievable they push forever and the position error
    // never settles -- so those are annealed to zero and only choose the branch.
    // Anneal the clearance terms toward a FLOOR, not to zero. At zero they cannot
    // hold a displacement they created: when the seed already satisfies the position
    // task (the usual case -- the warm start is the current posture) the position
    // descent contributes nothing, so once the clearance push dies the solution just
    // sits on the seed and the arm stops dodging. A residual keeps the displacement.
    // Termination is still safe because a null-space push does not move the tool, so
    // the final position tolerance is unaffected by it.
    //
    // `progress` is the UNFLOORED anneal fraction. Terminating on `w` instead was a
    // silent 40x realtime regression: once the floor was added, `w >= kClearanceFloor`
    // always held, so every guard written as `w < floor` became unreachable and every
    // descent ran the full iteration budget.
    const double progress = static_cast<double>(it) / cfg_.ik_max_iters;
    const double w = std::max(kClearanceFloor, 1.0 - progress);
    Eigen::VectorXd sec = Eigen::VectorXd::Zero(nv_);
    if (use_secondary) {
      // Constrain joint_1 ONLY. Driving a measured elbow azimuth instead would make
      // base facing and obstacle avoidance fight over the same quantity -- avoidance
      // reconfigures the elbow -- and the arm stops dodging entirely.
      if (face) sec(0) += cfg_.ik_base_face_gain * wrap_pi(base_yaw_for(goal_az) - q(0));
      sec += joint_limit_gradient(q);
      if (w > 1e-3) {
        Eigen::VectorXd clear = environment_gradient(q);
        if (obstacle_enabled_) clear += cfg_.obstacle_gain * obstacle_gradient(q);
        sec += w * clear;
      }
      if (sec.norm() > 0.5) sec *= 0.5 / sec.norm();  // never let it outrun the task
    }
    Eigen::VectorXd dq = Jinv * err + (In - Jinv * Jp) * sec;

    // Nothing is moving any more: the descent has reached a fixed point. Either it
    // converged, or this branch is stuck in a local minimum / against the workspace
    // boundary -- and in both cases the remaining iterations cannot change the
    // answer. This is the guard that keeps an unreachable goal, which burns all 26
    // seeds, inside the server's 20 ms tick.
    if (dq.norm() < 1e-5) break;
    // Holding the clearance weight at a floor means those terms never stop pushing,
    // so `dq` above does not fully collapse once the tool is on target -- the
    // solution keeps sliding along the null space. Give that refinement a bounded
    // budget rather than letting it run out the whole iteration count.
    if (err_norm < cfg_.ik_tol && ++settling > kSettleIters) break;

    if (dq.norm() > 0.2) dq *= 0.2 / dq.norm();  // keep the linearization valid
    q += dq;
    clamp_to_limits(q);
  }
  err_norm = (goal - fk_.body_pose(q).translation()).norm();
  if (err_norm >= cfg_.ik_tol) return std::nullopt;
  // The null-space term steers away from the scenery but cannot guarantee it, so a
  // posture that still ends up inside the table is rejected outright: better to
  // report the point as unreachable than to plan a move that drives into a contact.
  // Adjudicated by the plant's real meshes, not the coarse analytic model.
  if (posture_collides(q)) return std::nullopt;
  return q;
}

// A single damped-least-squares descent is greedy: for a target out over the table
// it walks the forearm straight DOWN through the tabletop, and the null-space
// avoidance term cannot climb back out once it is inside. So sweep a set of seeds
// across the redundant configuration space and keep the best branch that both
// reaches the goal and clears the scenery -- the same random-restart idea the M4
// solver uses, but with deterministic seeds so the planned posture is reproducible.
std::optional<Eigen::VectorXd> RedundancyController::solve_reach_posture(
    const Eigen::Vector3d& goal, const Eigen::VectorXd& seed,
    const Eigen::VectorXd* path_from) const {
  Eigen::VectorXd home(nv_);
  for (int i = 0; i < nv_ && i < 7; ++i) home(i) = kQHome[i];
  const bool face = goal.head<2>().norm() > cfg_.ik_base_face_min_radius;
  const double goal_az = std::atan2(goal.y(), goal.x());

  // A candidate is good enough to stop on when the arm actually POINTS at the target.
  // Taking the FIRST converged branch instead lands on whichever one the seed happened
  // to fall into, which routinely leaves the arm reaching sideways across itself.
  // Scored on the arm's world direction, not on q(0) -- see arm_azimuth().
  std::optional<Eigen::VectorXd> best;
  double best_miss = 1e9;
  auto consider = [&](const Eigen::VectorXd& s) {
    auto q = solve_reach_once(goal, s);
    if (!q) return false;
    // Prefer a branch the arm can actually GET to. A posture is only useful if the
    // straight joint-space move to it is clear as well.
    const bool clear = !path_from || !path_collides(*path_from, *q);
    double miss = face ? std::abs(wrap_pi(base_yaw_for(goal_az) - (*q)(0))) : 0.0;
    if (!clear) miss += 10.0;  // usable only if nothing better turns up
    if (miss < best_miss) {
      best_miss = miss;
      best = q;
    }
    return best_miss < 0.15;  // reachable AND facing the target: stop searching
  };

  // The warm start first, so a small goal change keeps the current posture and the
  // arm does not reconfigure for no reason; then home; then restarts spread across
  // the redundant configuration space. The RNG is a fixed-seed LCG rather than a real
  // one so the planned posture is reproducible run to run -- a planner that picks a
  // different branch each time it is asked the same question is unusable.
  if (consider(seed) || consider(home)) return best;

  // A goal outside the arm's geometric reach cannot be solved from ANY seed, so do
  // not spend the restart budget proving it -- that alone cost ~39 ms in a single
  // tick against a 20 ms realtime budget, i.e. a visible hitch whenever the ball was
  // dragged out of the workspace.
  if (goal.norm() > max_reach_) return best;

  std::uint32_t rng = 0x9e3779b9u;
  auto uniform = [&rng]() {  // [-1, 1)
    rng = rng * 1664525u + 1013904223u;
    return static_cast<double>(rng >> 8) / 8388608.0 - 1.0;
  };
  for (int restart = 0; restart < kIkRestarts; ++restart) {
    Eigen::VectorXd s = home;
    // Start already facing the target.
    if (face) s(0) = base_yaw_for(goal_az);
    // Only the first five joints change the arm's overall shape; the last two just
    // spin the tool, which a position-only task does not care about.
    for (int i = 1; i < nv_ && i < 5; ++i) s(i) += 1.4 * uniform();
    clamp_to_limits(s);
    if (consider(s)) return best;
  }
  return best;  // nullopt if nothing converged at all
}

// Same tool position, different elbow: step along the position task's kinematic
// null space, then re-project onto the exact tool point with IK. Used only by the
// optional redundancy demo.
std::optional<Eigen::VectorXd> RedundancyController::next_sweep_posture() const {
  const Eigen::VectorXd q = q_ref_;
  const Eigen::Vector3d tip = fk_.body_pose(q).translation();
  const Eigen::MatrixXd Jp =
      fk_.body_pose(q).rotation().matrix() * klib::body_jacobian(robot_, q).bottomRows(3);
  const Eigen::MatrixXd Jinv =
      Jp.transpose() * (Jp * Jp.transpose() + 1e-6 * Eigen::Matrix3d::Identity()).inverse();
  const Eigen::MatrixXd N = Eigen::MatrixXd::Identity(nv_, nv_) - Jinv * Jp;

  // Excite the elbow (joint_3) and the wrist roll (joint_5); joint_1 is left out so
  // the base keeps facing the target instead of yawing back and forth.
  Eigen::VectorXd pref = Eigen::VectorXd::Zero(nv_);
  pref(2) = 1.0;
  pref(4) = 0.5;
  Eigen::VectorXd dir = N * pref;
  if (dir.norm() < 1e-6) return std::nullopt;
  dir *= sweep_dir_ * kSweepStep / dir.norm();

  Eigen::VectorXd seed = q + dir;
  clamp_to_limits(seed);
  // solve_reach_once, NOT the full multi-seed search: this has to stay in the CURRENT
  // branch and land near `seed`. The restart search optimises for base facing and will
  // happily return a completely different posture with the same tool position, which
  // as a trajectory endpoint means the arm swings across itself instead of sweeping.
  return solve_reach_once(tip, seed, /*use_secondary=*/false);
}

// Build a synchronised, jerk-limited profile from the current reference to q_goal.
// A single scalar path parameter s runs 0 -> 1 along the straight joint-space line,
// and its velocity/acceleration/jerk caps are the tightest per-joint limit divided
// by that joint's travel. Every joint therefore respects its own limit and they all
// start and stop together -- the classic industrial joint move.
void RedundancyController::plan_to(const Eigen::VectorXd& q_goal) {
  q_start_ = q_ref_;
  // Take the short way round on continuous joints, and restate the goal in the same
  // unwrapped terms so the profile below stays a straight line to it.
  const Eigen::VectorXd dq = joint_delta(q_start_, q_goal);
  q_goal_ = q_start_ + dq;
  const double n = dq.norm();
  if (n < 1e-6) {
    s_ = 1.0;
    sd_ = sdd_ = 0.0;
    moving_ = false;
    qd_ref_.setZero();
    qdd_ref_.setZero();
    return;
  }

  const double scale = std::clamp(cfg_.speed_scale, 0.05, 1.0);
  const Eigen::VectorXd vmax = to_vec(cfg_.joint_vel_max, nv_) * scale;
  const Eigen::VectorXd amax = to_vec(cfg_.joint_acc_max, nv_) * scale;
  const Eigen::VectorXd jmax = to_vec(cfg_.joint_jerk_max, nv_) * scale;
  sd_max_ = sdd_max_ = sjerk_max_ = 1e9;
  for (int i = 0; i < nv_; ++i) {
    const double a = std::abs(dq(i));
    if (a < 1e-9) continue;
    sd_max_ = std::min(sd_max_, vmax(i) / a);
    sdd_max_ = std::min(sdd_max_, amax(i) / a);
    sjerk_max_ = std::min(sjerk_max_, jmax(i) / a);
  }

  // Carry the reference's current joint velocity AND acceleration into the new plan,
  // each projected onto the new direction, so a replan blends instead of stepping.
  // Zeroing them here instead leaves a jerk spike at every replan -- the same thing
  // the braking segment in replan() exists to avoid.
  sd_ = std::clamp(qd_ref_.dot(dq) / (n * n), 0.0, sd_max_);
  sdd_ = std::clamp(qdd_ref_.dot(dq) / (n * n), -sdd_max_, sdd_max_);
  s_ = 0.0;
  moving_ = true;
}

void RedundancyController::advance_trajectory(double dt) {
  if (!moving_) {
    q_ref_ = q_goal_;
    qd_ref_.setZero();
    qdd_ref_.setZero();
    return;
  }
  // Time-optimal double integrator on s: accelerate while there is still room to
  // brake to a stop, otherwise brake. The braking distance carries an extra term for
  // the time the jerk limit takes to reverse the acceleration, without which the
  // profile consistently overshoots the goal.
  const double brake = sd_ * sd_ / (2.0 * sdd_max_) + sd_ * sdd_max_ / (2.0 * sjerk_max_);
  double sdd_cmd = (1.0 - s_ > brake) ? sdd_max_ : -sdd_max_;
  if (sd_ >= sd_max_ && sdd_cmd > 0.0) sdd_cmd = 0.0;  // cruise at the velocity cap
  sdd_ = std::clamp(sdd_cmd, sdd_ - sjerk_max_ * dt, sdd_ + sjerk_max_ * dt);
  sd_ = std::clamp(sd_ + sdd_ * dt, 0.0, sd_max_);
  s_ += sd_ * dt;

  const Eigen::VectorXd dq = q_goal_ - q_start_;
  if (s_ >= 1.0 || (sd_ <= 1e-9 && 1.0 - s_ < 1e-6)) {
    s_ = 1.0;
    sd_ = sdd_ = 0.0;
    moving_ = false;  // arrived: the arm comes to a complete stop
  }
  q_ref_ = q_start_ + s_ * dq;
  qd_ref_ = sd_ * dq;
  qdd_ref_ = sdd_ * dq;
}

void RedundancyController::replan() {
  replan_pending_ = false;
  planned_obstacle_ = obstacle_;
  stall_timer_ = 0.0;

  // A retarget while the arm is already moving is served as TWO moves: first brake
  // to a stop along the heading it is already on, then set off for the new goal.
  // Splicing the new direction straight in would step the reference velocity
  // sideways -- continuous in position but not in velocity, which the tracker turns
  // into a jolt. This is what an industrial controller does at an unblended via point.
  if (moving_ && qd_ref_.norm() > 1e-3) {
    const Eigen::VectorXd dq_now = q_goal_ - q_start_;
    if (dq_now.norm() > 1e-9) {
      // Where the current move coasts to a stop under its own deceleration limit.
      const double s_stop = std::min(1.0, s_ + sd_ * sd_ / (2.0 * sdd_max_));
      const Eigen::VectorXd stop_at = q_start_ + s_stop * dq_now;
      braking_ = true;  // step() re-arms the replan once this segment finishes
      plan_to(stop_at);  // same heading, so the speed carries over untouched
      return;
    }
  }
  braking_ = false;

  // Warm-start from the last accepted posture so the solution stays in the same
  // branch and the arm does not flip elbow-up/elbow-down between goals.
  auto sol = solve_reach_posture(goal_, q_ik_, &q_ref_);
  bool exact = sol.has_value();

  if (!exact) {
    // The ball is out of reach, or the only postures that get there go through the
    // scenery. Bisect along the line from the current tip to the ball for the
    // furthest point that IS solvable, and go there -- the arm still reaches out
    // toward the ball as far as it honestly can, instead of freezing where it stands.
    // solve_reach_once, NOT the full restart search: this walks a line outward from a
    // posture the arm is already in, so a warm start is exactly right -- and running
    // 40 restarts at each of 12 bisection steps costs over a second in a single tick.
    const Eigen::Vector3d here = fk_.body_pose(q_ref_).translation();
    double lo = 0.0, hi = 1.0;  // fraction of the way from `here` to the ball
    Eigen::VectorXd best = q_ref_;
    for (int it = 0; it < 10; ++it) {
      const double mid = 0.5 * (lo + hi);
      if (auto s = solve_reach_once(here + mid * (goal_ - here), best)) {
        best = *s;
        lo = mid;
      } else {
        hi = mid;
      }
    }
    sol = best;
    phase_ = Phase::Unreachable;
  }

  q_ik_ = *sol;
  plan_to(q_ik_);
  if (exact) phase_ = moving_ ? Phase::Moving : Phase::Holding;
}

void RedundancyController::step() {
  const double dt = m_->opt.timestep;
  const Eigen::VectorXd q = read_joints(d_->qpos, nv_);
  const Eigen::VectorXd qd = read_joints(d_->qvel, nv_);

  // --- plan (only when something actually changed) --------------------------
  if (replan_pending_) replan();

  // --- optional redundancy demo: chain small planned null-space steps --------
  // Only while genuinely parked on the ball -- not while stuck or short of an
  // unreachable target, where reconfiguring would just grind against whatever is
  // in the way.
  if (sweep_enabled_ && !moving_ && phase_ == Phase::Holding) {
    if (auto nxt = next_sweep_posture()) {
      plan_to(*nxt);
      if (!moving_) sweep_dir_ = -sweep_dir_;  // nowhere left to go: turn around
    } else {
      sweep_dir_ = -sweep_dir_;
    }
  }

  // --- trajectory -----------------------------------------------------------
  advance_trajectory(dt);
  // The pre-retarget stop segment has finished; now plan the move the user asked for.
  if (braking_ && !moving_) {
    braking_ = false;
    replan_pending_ = true;
  }

  // --- tracking: computed torque with full feedforward ----------------------
  // tau = M(q) * (qdd_ref + Kd*(qd_ref - qd) + Kp*(q_ref - q) + Ki*integ)
  //       + C(q,qd)*qd + g(q) + friction feedforward
  // so the joint error obeys e'' + Kd e' + Kp e = 0 -- decoupled and, at
  // damping_ratio = 1, critically damped: it converges without overshoot.
  const Eigen::VectorXd bias =
      klib::coriolis_term(robot_, q, qd) + klib::gravity_term(robot_, q);
  Eigen::MatrixXd M = klib::mass_matrix(robot_, q);
  M.diagonal().array() += cfg_.armature;  // reflected rotor inertia the plant integrates

  const double Kp = cfg_.bandwidth * cfg_.bandwidth;
  const double Kd = 2.0 * cfg_.damping_ratio * cfg_.bandwidth;
  const Eigen::VectorXd e = q_ref_ - q;
  const Eigen::VectorXd ed = qd_ref_ - qd;

  // Anti-windup by conditional integration: a joint whose command is already
  // saturated does not charge its integral further (only unwinding is allowed).
  // Without this the integral climbed to ~90 N.m against the table and pinned the
  // arm there with every actuator railed.
  for (int i = 0; i < nv_; ++i) {
    const bool pushing_further = sat_(i) != 0.0 && sat_(i) * e(i) > 0.0;
    if (!pushing_further) integ_j_(i) += e(i) * dt;
    integ_j_(i) = std::clamp(integ_j_(i), -cfg_.integral_clamp, cfg_.integral_clamp);
  }

  // Feed the plant's own viscous damping and Coulomb friction forward from the
  // REFERENCE velocity (not the measured one, which would be positive feedback).
  // Compensating Coulomb friction here is what removes the stick-slip hunting that
  // an integral alone produces around the setpoint.
  Eigen::VectorXd fric(nv_);
  for (int i = 0; i < nv_; ++i)
    fric(i) = cfg_.friction_ff * (jnt_damping_(i) * qd_ref_(i) +
                                  jnt_friction_(i) * std::tanh(qd_ref_(i) / 0.01));

  Eigen::VectorXd tau =
      M * (qdd_ref_ + Kd * ed + Kp * e + cfg_.ki * integ_j_) + bias + fric;

  // --- saturation, matching what the plant actually applies -----------------
  for (int i = 0; i < nv_; ++i) {
    const double lim = tau_lim_(i);
    if (tau(i) > lim) { tau(i) = lim; sat_(i) = 1.0; }
    else if (tau(i) < -lim) { tau(i) = -lim; sat_(i) = -1.0; }
    else sat_(i) = 0.0;
  }

  // --- protective stop ------------------------------------------------------
  // The plan is finished but the arm is not where it was told to be and is not
  // moving: it is stuck against the table or a joint limit. Give up on the goal and
  // hold the current posture instead of leaning on the actuators indefinitely.
  if (!moving_ && phase_ != Phase::Unreachable) {
    const bool stuck = e.norm() > cfg_.stall_err && qd.norm() < 0.02;
    stall_timer_ = stuck ? stall_timer_ + dt : 0.0;
    if (stall_timer_ > cfg_.stall_time) {
      q_ref_ = q_goal_ = q;
      qd_ref_.setZero();
      qdd_ref_.setZero();
      integ_j_.setZero();
      phase_ = Phase::Stalled;
      stall_timer_ = 0.0;
    }
  }
  if (phase_ == Phase::Moving && !moving_) phase_ = Phase::Holding;

  for (int i = 0; i < m_->nu; ++i) d_->ctrl[i] = (i < tau.size()) ? tau[i] : 0.0;
  mj_step(m_, d_);  // MuJoCo integrates; our torques drive the plant
  last_tau_ = tau;
}

State RedundancyController::get_state() const {
  State s;
  const Eigen::VectorXd q = read_joints(d_->qpos, nv_);
  const Eigen::VectorXd qd = read_joints(d_->qvel, nv_);

  const klib::SE3 T = fk_.body_pose(q);  // OUR FK (M2)
  const Eigen::Vector3d tip = T.translation();
  const Eigen::Quaterniond quat(T.rotation().matrix());
  const Eigen::Vector3d elbow = fk_.link_poses(q)[kElbowLink].translation();

  s.q.assign(d_->qpos, d_->qpos + std::min(7, static_cast<int>(m_->nq)));
  for (int i = 0; i < 3; ++i) {
    s.tip[i] = tip[i];
    s.target[i] = goal_[i];
    s.obstacle[i] = obstacle_[i];
    s.elbow[i] = elbow[i];
  }
  switch (phase_) {
    case Phase::Moving: s.phase = "moving"; break;
    case Phase::Stalled: s.phase = "stalled"; break;
    case Phase::Unreachable: s.phase = "unreachable"; break;
    default: s.phase = "holding"; break;
  }
  s.obstacle_on = obstacle_enabled_;
  s.sweep_on = sweep_enabled_;
  s.fk_quat = {quat.w(), quat.x(), quat.y(), quat.z()};
  s.tip_err_mm = (goal_ - tip).norm() * 1000.0;
  s.clearance_m = std::max(0.0, nearest_clearance(q));
  s.obstacle_radius = cfg_.obstacle_radius;
  s.manip = klib::manipulability(robot_, q);  // OUR manipulability (M3)

  const Eigen::MatrixXd Jp =
      T.rotation().matrix() * klib::body_jacobian(robot_, q).bottomRows(3);
  s.tip_speed = (Jp * qd).norm();
  s.joint_speed_max = qd.cwiseAbs().maxCoeff();
  s.track_err_rad = (q_ref_ - q).norm();
  s.progress = s_;
  s.speed_scale = cfg_.speed_scale;
  s.eta_s = (moving_ && sd_ > 1e-6) ? (1.0 - s_) / sd_ : 0.0;

  // Live inverse-kinematics check (OUR DLS IK, M4): from a FIXED home seed,
  // invert the current tool pose and report the residual (an honest inversion).
  Eigen::VectorXd seed(nv_);
  for (int i = 0; i < nv_ && i < 7; ++i) seed(i) = kQHome[i];
  if (auto sol = ik_.solve(T, seed)) {
    const klib::SE3 reached = fk_.body_pose(*sol);
    s.q_ik.assign(sol->data(), sol->data() + sol->size());
    s.ik_ok = true;
    s.ik_pos_mm = (reached.translation() - tip).norm() * 1000.0;
    s.ik_ori_deg = (reached.inverse() * T).log().head<3>().norm() * kRad2Deg;
    s.ik_joint_dist = (*sol - q).norm();
  }

  s.tau.assign(last_tau_.data(), last_tau_.data() + last_tau_.size());
  return s;
}

}  // namespace webviz
