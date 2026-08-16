#!/usr/bin/env bash
# Build and run the kinematics micro-benchmarks, emitting JSON next to the binary.
#
# Needs Google Benchmark (brew install google-benchmark / apt install libbenchmark-dev).
# The top-level CMakeLists skips benchmarks/ when it is absent, so check for that
# explicitly here rather than failing later with a confusing "no such file".
set -euo pipefail
cd "$(dirname "$0")/.."

BUILD=build

cmake -S . -B "$BUILD" -DCMAKE_BUILD_TYPE=Release >/dev/null
if ! cmake --build "$BUILD" --target bench_kinematics >/dev/null 2>&1; then
  echo "bench_kinematics did not build -- is Google Benchmark installed?" >&2
  echo "  macOS:  brew install google-benchmark" >&2
  echo "  Debian: apt install libbenchmark-dev" >&2
  exit 1
fi

"$BUILD/benchmarks/bench_kinematics" \
  --benchmark_min_time=0.5s \
  --benchmark_format=json --benchmark_out="$BUILD/bench_kinematics.json"
echo "wrote $BUILD/bench_kinematics.json"
