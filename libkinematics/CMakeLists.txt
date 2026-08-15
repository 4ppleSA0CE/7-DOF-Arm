cmake_minimum_required(VERSION 3.16)
project(libkinematics CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

# Dependencies are discovered by path rather than by CMake config package. Eigen's
# config file uses SameMajorVersion compatibility, so `find_package(Eigen3 3.4)`
# rejects Eigen 5.x outright, and yaml-cpp has exported two different target names
# across releases. Both libraries are simple enough that locating the headers (and
# one library) sidesteps the whole problem. Matches webviz/server/CMakeLists.txt.
find_path(EIGEN3_INCLUDE_DIR Eigen/Core
  PATHS /opt/homebrew/include/eigen3 /usr/local/include/eigen3 /usr/include/eigen3
  REQUIRED)
find_path(YAMLCPP_INCLUDE_DIR yaml-cpp/yaml.h
  PATHS /opt/homebrew/include /usr/local/include /usr/include
  REQUIRED)
find_library(YAMLCPP_LIBRARY NAMES yaml-cpp
  PATHS /opt/homebrew/lib /usr/local/lib /usr/lib
  REQUIRED)

add_library(kinematics
  src/math/so3.cpp
  src/math/se3.cpp
  src/robot.cpp
  src/fk.cpp
  src/jacobian.cpp
  src/ik.cpp
  src/dynamics.cpp
  src/null_space.cpp
)
target_include_directories(kinematics PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include ${EIGEN3_INCLUDE_DIR} ${YAMLCPP_INCLUDE_DIR})
target_link_libraries(kinematics PUBLIC ${YAMLCPP_LIBRARY})
set_target_properties(kinematics PROPERTIES POSITION_INDEPENDENT_CODE ON)
# Instrument hot paths so the no-heap-allocation tests can assert via
# Eigen::internal::set_is_malloc_allowed(false). Note this only has teeth in a
# Debug build: NDEBUG defines EIGEN_NO_DEBUG, which compiles the guard away.
target_compile_definitions(kinematics PUBLIC EIGEN_RUNTIME_NO_MALLOC)
add_library(libkinematics::kinematics ALIAS kinematics)

# Absolute paths to the model description and the reference fixtures.
set(CFG "${CMAKE_CURRENT_SOURCE_DIR}/../kinova_gen2_description/config")
set(KINOVA_SCREWS_YAML "${CFG}/kinova_gen2_screws.yaml")
set(KINOVA_DYNAMICS_YAML "${CFG}/kinova_gen2_dynamics.yaml")
set(FK_REFERENCE_CSV "${CMAKE_CURRENT_SOURCE_DIR}/test/fk_reference.csv")
set(DYN_REFERENCE_CSV "${CMAKE_CURRENT_SOURCE_DIR}/test/dyn_reference.csv")
set(MASS_REFERENCE_CSV "${CMAKE_CURRENT_SOURCE_DIR}/test/mass_reference.csv")

# CLI to eyeball FK/IK/redundancy numbers from the terminal.
add_executable(kin_report tools/kin_report.cpp)
target_link_libraries(kin_report PRIVATE kinematics)
target_compile_definitions(kin_report PRIVATE KINOVA_SCREWS_YAML="${KINOVA_SCREWS_YAML}")

# Writes learn/test/reference.json, the oracle the JS lecture engine is checked
# against. Not a test: run it by hand when the model or an algorithm changes.
add_executable(gen_learn_reference tools/gen_learn_reference.cpp)
target_link_libraries(gen_learn_reference PRIVATE kinematics)
target_compile_definitions(gen_learn_reference PRIVATE
  KINOVA_SCREWS_YAML="${KINOVA_SCREWS_YAML}" KINOVA_DYNAMICS_YAML="${KINOVA_DYNAMICS_YAML}")

enable_testing()
find_package(GTest QUIET)
if(NOT GTest_FOUND)
  include(FetchContent)
  FetchContent_Declare(googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz)
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
endif()
file(GLOB TEST_SOURCES test/test_*.cpp)
foreach(test_src ${TEST_SOURCES})
  get_filename_component(test_name ${test_src} NAME_WE)
  add_executable(${test_name} ${test_src})
  target_link_libraries(${test_name} PRIVATE kinematics GTest::gtest_main)
  target_compile_definitions(${test_name} PRIVATE
    KINOVA_SCREWS_YAML="${KINOVA_SCREWS_YAML}" KINOVA_DYNAMICS_YAML="${KINOVA_DYNAMICS_YAML}"
    FK_REFERENCE_CSV="${FK_REFERENCE_CSV}" DYN_REFERENCE_CSV="${DYN_REFERENCE_CSV}"
    MASS_REFERENCE_CSV="${MASS_REFERENCE_CSV}")
  add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()
