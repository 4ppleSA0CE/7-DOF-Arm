#pragma once
// Robot kinematic description for body-form PoE. Twist order [omega; v].
#include <array>
#include <string>
#include <vector>

#include "libkinematics/math/se3.hpp"
#include "libkinematics/math/types.hpp"

namespace klib {

// Rigid-body inertia of a link, expressed in that link's frame.
struct LinkInertia {
  double mass = 0.0;
  Eigen::Vector3d com = Eigen::Vector3d::Zero();          // center of mass
  Eigen::Matrix3d inertia_com = Eigen::Matrix3d::Zero();  // about com
};

struct Robot {
  // Kinematics (body-form PoE).
  std::vector<Twist> body_screw_axes;             // B_i, [omega; v], tool frame at home
  SE3 home_pose_M;                                // M: tool pose at q = 0
  std::vector<SE3> link_home_frames;              // world pose of each jointed body at q = 0
  std::vector<std::array<double, 2>> joint_limits;  // [min, max] rad per joint

  // Dynamics (loaded separately; empty until load_dynamics_yaml is called).
  std::vector<Twist> link_screw_axes;   // A_i, screw of joint i in link frame i
  std::vector<SE3> relative_home;       // M_{i-1,i} for i=1..n, then M_{n,tool} (size n+1)
  std::vector<LinkInertia> link_inertias;

  int dof() const noexcept { return static_cast<int>(body_screw_axes.size()); }
  bool has_dynamics() const noexcept { return !link_inertias.empty(); }
};

// Load a Robot from a screws YAML (see kinova_gen2_description/config/*.yaml).
Robot load_robot_yaml(const std::string& path);

// Augment an existing Robot with the dynamics fields from a dynamics YAML.
Robot load_dynamics_yaml(Robot robot, const std::string& path);

}  // namespace klib
