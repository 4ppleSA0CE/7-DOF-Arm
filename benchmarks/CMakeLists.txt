cmake_minimum_required(VERSION 3.16)
project(benchmarks CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Built as part of the top-level project, which supplies the `kinematics` target
# and has already confirmed Google Benchmark is present.
find_package(benchmark REQUIRED)

set(CFG "${CMAKE_CURRENT_SOURCE_DIR}/../kinova_gen2_description/config")

add_executable(bench_kinematics bench_kinematics.cpp)
target_link_libraries(bench_kinematics PRIVATE kinematics benchmark::benchmark)
target_compile_definitions(bench_kinematics PRIVATE
  KINOVA_SCREWS_YAML="${CFG}/kinova_gen2_screws.yaml"
  KINOVA_DYNAMICS_YAML="${CFG}/kinova_gen2_dynamics.yaml")
