// WebSocket sim server: steps MuJoCo in realtime and streams state to browsers.
//
// Single-threaded, like the asyncio prototype it replaces. Each 20 ms tick:
// pump the socket (accept clients, apply any inbound commands), advance the sim
// 20 x 1 ms substeps, then broadcast the latest state at 50 Hz. No locks needed
// -- one thread owns the controller and the server.
#include <chrono>
#include <cstdio>
#include <string>
#include <thread>

#include <nlohmann/json.hpp>

#include "redundancy_controller.hpp"
#include "ws_server.hpp"

using json = nlohmann::json;

namespace {
constexpr double kTick = 0.02;  // 50 Hz broadcast
constexpr int kSubsteps = 20;   // 20 x 1 ms = 20 ms sim per tick (realtime)

json state_to_json(const webviz::State& s) {
  return json{
      {"q", s.q},
      {"tip", s.tip},
      {"target", s.target},
      {"obstacle", s.obstacle},
      {"elbow", s.elbow},
      {"phase", s.phase},
      {"obstacle_on", s.obstacle_on},
      {"sweep_on", s.sweep_on},
      {"tip_err_mm", s.tip_err_mm},
      {"clearance_m", s.clearance_m},
      {"obstacle_radius", s.obstacle_radius},
      {"manip", s.manip},
      {"tau", s.tau},
      {"progress", s.progress},
      {"tip_speed", s.tip_speed},
      {"joint_speed_max", s.joint_speed_max},
      {"track_err_rad", s.track_err_rad},
      {"speed_scale", s.speed_scale},
      {"eta_s", s.eta_s},
      {"fk_quat", s.fk_quat},
      {"q_ik", s.q_ik},
      {"ik_ok", s.ik_ok},
      {"ik_pos_mm", s.ik_pos_mm},
      {"ik_ori_deg", s.ik_ori_deg},
      {"ik_joint_dist", s.ik_joint_dist},
  };
}

// Pull a length-3 array out of a JSON field, returning false if it isn't one.
bool as_vec3(const json& j, Eigen::Vector3d& out) {
  if (!j.is_array() || j.size() != 3) return false;
  for (int i = 0; i < 3; ++i) {
    if (!j[i].is_number()) return false;
    out[i] = j[i].get<double>();
  }
  return true;
}
}  // namespace

int main(int argc, char** argv) {
  // Tuning + port come from webviz/config.yaml (or the path given as argv[1], or
  // $WEBVIZ_CONFIG). Nothing here needs a rebuild to re-tune.
  std::string cfg_path = (argc > 1) ? argv[1] : "";
  std::string loaded_from;
  const webviz::Config cfg = webviz::load_config(cfg_path, &loaded_from);
  std::printf("config: %s\n", loaded_from.empty() ? "(built-in defaults)" : loaded_from.c_str());

  webviz::RedundancyController ctrl(cfg);
  wsx::WsServer srv(static_cast<uint16_t>(cfg.port));
  std::printf("sim_server on ws://localhost:%d\n", cfg.port);
  std::fflush(stdout);

  // Apply one inbound browser command: drag the green ball (target) or red ball
  // (obstacle), toggle a mode, or reset. Each goal change triggers exactly one replan.
  auto on_message = [&ctrl](const std::string& raw) {
    json msg = json::parse(raw, nullptr, /*allow_exceptions=*/false);
    if (msg.is_discarded() || !msg.is_object()) return;
    Eigen::Vector3d v;
    if (msg.contains("target") && as_vec3(msg["target"], v)) ctrl.set_target(v);
    if (msg.contains("obstacle") && as_vec3(msg["obstacle"], v)) ctrl.set_obstacle(v);
    if (msg.contains("obstacle_on") && msg["obstacle_on"].is_boolean())
      ctrl.set_obstacle_enabled(msg["obstacle_on"].get<bool>());
    if (msg.contains("sweep_on") && msg["sweep_on"].is_boolean())
      ctrl.set_posture_sweep(msg["sweep_on"].get<bool>());
    if (msg.contains("speed_scale") && msg["speed_scale"].is_number())
      ctrl.set_speed_scale(msg["speed_scale"].get<double>());
    if (msg.value("reset", false)) ctrl.reset();
  };

  // Schedule against an ABSOLUTE deadline rather than sleeping for the leftover slack.
  // sleep_for only guarantees a minimum, and the overshoot (measured up to ~13 ms
  // under load on this machine) is pure loss with the naive form: every tick starts
  // late and the sim quietly runs below realtime, so the arm looks slower than the
  // motion limits say it is. Advancing a fixed deadline absorbs jitter instead.
  using clock = std::chrono::steady_clock;
  const auto tick = std::chrono::duration_cast<clock::duration>(
      std::chrono::duration<double>(kTick));
  auto next = clock::now() + tick;
  for (;;) {
    srv.poll(0, on_message);  // accept + drain commands (non-blocking)
    for (int i = 0; i < kSubsteps; ++i) ctrl.step();
    if (srv.client_count() > 0) srv.broadcast(state_to_json(ctrl.get_state()).dump());

    const auto now = clock::now();
    if (now < next) {
      std::this_thread::sleep_until(next);
      next += tick;
    } else {
      // Fell behind (a slow replan, or the machine is loaded). Resynchronise instead
      // of trying to catch up, which would sprint the arm to make up lost time.
      next = now + tick;
    }
  }
}
