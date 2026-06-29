cmake_minimum_required(VERSION 3.16)
project(libkinematics CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(Eigen3 3.4 REQUIRED)
find_package(yaml-cpp REQUIRED)

# Sources grow as modules land.
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
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(kinematics PUBLIC Eigen3::Eigen yaml-cpp)
# Position-independent so the static lib can link into shared controller plugins.
set_target_properties(kinematics PROPERTIES POSITION_INDEPENDENT_CODE ON)
# Instrument hot paths so the no-heap-allocation tests can assert via
# Eigen::internal::set_is_malloc_allowed(false). Default flag is "allowed",
# so this is benign in production.
target_compile_definitions(kinematics PUBLIC EIGEN_RUNTIME_NO_MALLOC)

# Absolute paths to test data (config yaml lives in a sibling package).
set(CFG "${CMAKE_CURRENT_SOURCE_DIR}/../kinova_gen2_description/config")
set(KINOVA_SCREWS_YAML "${CFG}/kinova_gen2_screws.yaml")
set(KINOVA_DYNAMICS_YAML "${CFG}/kinova_gen2_dynamics.yaml")
set(FK_REFERENCE_CSV "${CMAKE_CURRENT_SOURCE_DIR}/test/fk_reference.csv")
set(DYN_REFERENCE_CSV "${CMAKE_CURRENT_SOURCE_DIR}/test/dyn_reference.csv")
set(MASS_REFERENCE_CSV "${CMAKE_CURRENT_SOURCE_DIR}/test/mass_reference.csv")

# --- tests: works under both ament (colcon) and plain ctest --------------
find_package(ament_cmake QUIET)

if(ament_cmake_FOUND)
  # Canonical path: colcon test --packages-select libkinematics
  install(DIRECTORY include/ DESTINATION include)
  install(TARGETS kinematics EXPORT export_kinematics
          ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
  ament_export_targets(export_kinematics HAS_LIBRARY_TARGET)
  ament_export_dependencies(Eigen3)

  if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    file(GLOB TEST_SOURCES test/test_*.cpp)
    foreach(test_src ${TEST_SOURCES})
      get_filename_component(test_name ${test_src} NAME_WE)
      ament_add_gtest(${test_name} ${test_src})
      target_link_libraries(${test_name} kinematics)
      target_compile_definitions(${test_name} PRIVATE
        KINOVA_SCREWS_YAML="${KINOVA_SCREWS_YAML}" KINOVA_DYNAMICS_YAML="${KINOVA_DYNAMICS_YAML}"
        FK_REFERENCE_CSV="${FK_REFERENCE_CSV}" DYN_REFERENCE_CSV="${DYN_REFERENCE_CSV}"
        MASS_REFERENCE_CSV="${MASS_REFERENCE_CSV}")
    endforeach()
  endif()
  ament_package()
else()
  # Standalone path: cmake -S libkinematics -B build && ctest --test-dir build
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
    target_link_libraries(${test_name} kinematics GTest::gtest GTest::gtest_main)
    target_compile_definitions(${test_name} PRIVATE
      KINOVA_SCREWS_YAML="${KINOVA_SCREWS_YAML}" KINOVA_DYNAMICS_YAML="${KINOVA_DYNAMICS_YAML}"
      FK_REFERENCE_CSV="${FK_REFERENCE_CSV}" DYN_REFERENCE_CSV="${DYN_REFERENCE_CSV}"
      MASS_REFERENCE_CSV="${MASS_REFERENCE_CSV}")
    add_test(NAME ${test_name} COMMAND ${test_name})
  endforeach()
endif()
