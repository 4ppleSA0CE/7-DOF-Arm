#!/usr/bin/env bash
# Builds + starts the C++ MuJoCo sim server, then the Vite dev server.
# Ctrl-C stops both. Open http://localhost:5173.
#
# Tuning (including the WebSocket port, default 8770) comes from
# webviz/config.yaml -- no rebuild needed to change it. Pass a different config
# file as the first argument.
set -euo pipefail
cd "$(dirname "$0")/.."

CONFIG=${1:-webviz/config.yaml}
BUILD=webviz/server/build

cmake -S webviz/server -B "$BUILD" -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build "$BUILD" --target sim_server

"$BUILD/sim_server" "$CONFIG" &
SIM=$!
trap 'kill $SIM 2>/dev/null || true' EXIT

cd webviz/app && npm run dev
