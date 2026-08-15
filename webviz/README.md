# webviz

Browser dashboard for the redundancy controller. See the [root
README](../README.md) for what the demo does and how the control pipeline works,
and [THEORY.md](../THEORY.md) for the maths behind it. This file covers only the
parts specific to running and configuring the server.

```
webviz/server/  C++    MuJoCo plant + controller, streams state at 50 Hz over a WebSocket
webviz/app/     React  three.js (urdf-loader) render + MUI dashboard + chart.js
```

The server is plain CMake, Eigen, MuJoCo, yaml-cpp and nlohmann/json, with no
ROS. It links MuJoCo from an explicit install (`MUJOCO_DIR`, or `/opt/mujoco`)
or, failing that, from the library bundled inside the `pip install mujoco`
package. WebSocket framing is a vendored RFC6455 header.

## Run

```bash
( cd webviz/app && npm install )   # one-time: front-end deps
./scripts/run_webviz.sh            # builds the C++ server, starts both halves
# open http://localhost:5173
```

To run the halves separately:

```bash
cmake -S webviz/server -B webviz/server/build && cmake --build webviz/server/build
webviz/server/build/sim_server
cd webviz/app && npm run dev       # in another shell
```

## Configuration

All tuning lives in [`config.yaml`](config.yaml) and is read at startup, so no
rebuild is needed to re-tune. It covers the WebSocket port, the per-joint
velocity, acceleration and jerk limits, the tracking gains, the IK settings,
obstacle avoidance and the protective stop. Every key is optional; anything you
leave out keeps its built-in default, and a value that would break the controller
(a non-positive limit, a zero iteration budget) is rejected with a message on
stderr and replaced by the default.

The server picks a config in this order: the path given as its first argument,
`$WEBVIZ_CONFIG`, the baked-in repo path, then `./config.yaml`.

```bash
webviz/server/build/sim_server                 # webviz/config.yaml
webviz/server/build/sim_server my-tuning.yaml  # explicit
WEBVIZ_CONFIG=slow.yaml webviz/server/build/sim_server
```

**Port.** The server defaults to **8770**, clear of **8765** (Foxglove's default).
If you change `server.port`, point the browser at the new one by editing
`VITE_WS_URL` in [`app/.env`](app/.env).

## Wire protocol

The browser opens a WebSocket to `ws://localhost:8770`. The server broadcasts one
JSON object per tick at 50 Hz, carrying joint angles, tool and marker positions,
phase, tip error, clearance, manipulability, torques, progress and ETA, plus a
live inverse-kinematics readout. Dragging a marker sends `{target: [...]}` or
`{obstacle: [...]}` back, which triggers exactly one replan. `state_to_json` in
`server/sim_server.cpp` is the authoritative field list; `app/src/types.ts`
mirrors it.

## Checks

```bash
ctest --test-dir webviz/server/build     # test_controller + test_reach
webviz/server/build/diag_motion          # motion-quality profile (reporting tool)
```

`test_reach` is the accuracy battery: base facing, a spread of targets,
out-of-reach handling, obstacle avoidance, recovery, and continuous-joint
wrapping. `test_controller` asserts the null-space demo keeps the tool pinned
while the elbow sweeps. `diag_motion` prints peak tip speed, acceleration and
jerk, peak joint rate, actuator saturation, overshoot, settling time and residual
motion per target; it reports rather than passes or fails.
