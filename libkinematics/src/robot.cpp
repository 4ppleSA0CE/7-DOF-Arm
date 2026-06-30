#include "libkinematics/robot.hpp"

#include <yaml-cpp/yaml.h>

#include <Eigen/Geometry>
#include <stdexcept>
#include <vector>

namespace klib {

namespace {
SE3 poseFromNode(const YAML::Node& n) {
  const auto p = n["position"].as<std::vector<double>>();
  const auto q = n["quaternion_wxyz"].as<std::vector<double>>();
  Eigen::Quaterniond quat(q[0], q[1], q[2], q[3]);  // (w, x, y, z)
  quat.normalize();
  return SE3(SO3(quat.toRotationMatrix()), Eigen::Vector3d(p[0], p[1], p[2]));
}
}  // namespace

Robot load_robot_yaml(const std::string& path) {
  YAML::Node root = YAML::LoadFile(path);
  Robot r;
  r.home_pose_M = poseFromNode(root["home_pose"]);

  for (const auto& row : root["body_screw_axes"]) {
    const auto v = row.as<std::vector<double>>();
    if (v.size() != 6) throw std::runtime_error("body_screw_axes row must have 6 entries");
    Twist b;
    b << v[0], v[1], v[2], v[3], v[4], v[5];
    r.body_screw_axes.push_back(b);
  }
  for (const auto& f : root["link_home_frames"]) {
    r.link_home_frames.push_back(poseFromNode(f));
  }
  for (const auto& lim : root["joint_limits"]) {
    const auto v = lim.as<std::vector<double>>();
    r.joint_limits.push_back({v[0], v[1]});
  }
  if (r.link_home_frames.size() != r.body_screw_axes.size())
    throw std::runtime_error("link_home_frames count must equal joint count");
  return r;
}

Robot load_dynamics_yaml(Robot robot, const std::string& path) {
  YAML::Node root = YAML::LoadFile(path);

  for (const auto& row : root["link_screw_axes"]) {
    const auto v = row.as<std::vector<double>>();
    Twist a;
    a << v[0], v[1], v[2], v[3], v[4], v[5];
    robot.link_screw_axes.push_back(a);
  }
  for (const auto& f : root["relative_home_transforms"]) {
    robot.relative_home.push_back(poseFromNode(f));
  }
  for (const auto& li : root["link_inertias"]) {
    LinkInertia inertia;
    inertia.mass = li["mass"].as<double>();
    const auto c = li["com"].as<std::vector<double>>();
    inertia.com = Eigen::Vector3d(c[0], c[1], c[2]);
    const auto I = li["inertia_com"].as<std::vector<double>>();  // Ixx Iyy Izz Ixy Ixz Iyz
    inertia.inertia_com << I[0], I[3], I[4], I[3], I[1], I[5], I[4], I[5], I[2];
    robot.link_inertias.push_back(inertia);
  }
  const int n = robot.dof();
  if (static_cast<int>(robot.link_screw_axes.size()) != n ||
      static_cast<int>(robot.link_inertias.size()) != n ||
      static_cast<int>(robot.relative_home.size()) != n + 1) {
    throw std::runtime_error("dynamics yaml dimensions inconsistent with joint count");
  }
  return robot;
}

}  // namespace klib
