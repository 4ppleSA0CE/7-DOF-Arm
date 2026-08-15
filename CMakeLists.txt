cmake_minimum_required(VERSION 3.16)
project(redundant_arm_control CXX)

# One command builds the math library and its tests:
#   cmake -S . -B build && cmake --build build && ctest --test-dir build
#
# The browser demo under webviz/ is built separately by scripts/run_webviz.sh --
# it needs MuJoCo and does its own dylib rpath fixups, which are not worth
# entangling with a build whose only hard dependencies are Eigen and yaml-cpp.

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

enable_testing()
add_subdirectory(libkinematics)

# Micro-benchmarks need Google Benchmark; the library and its tests do not, so a
# missing dependency here should not fail the build a reviewer actually runs.
find_package(benchmark QUIET)
if(benchmark_FOUND)
  add_subdirectory(benchmarks)
else()
  message(STATUS "Google Benchmark not found -- skipping benchmarks/ "
                 "(brew install google-benchmark)")
endif()
