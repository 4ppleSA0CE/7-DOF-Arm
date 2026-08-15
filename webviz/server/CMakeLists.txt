cmake_minimum_required(VERSION 3.16)
project(webviz_sim_server CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra)

# --- Eigen (header-only) ------------------------------------------------------
find_path(EIGEN3_INCLUDE_DIR Eigen/Core
  PATHS /opt/homebrew/include/eigen3 /usr/local/include/eigen3 /usr/include/eigen3
  REQUIRED)

# --- nlohmann/json (header-only) ----------------------------------------------
find_path(JSON_INCLUDE_DIR nlohmann/json.hpp
  PATHS /opt/homebrew/include /usr/local/include /usr/include
  REQUIRED)

# --- MuJoCo -------------------------------------------------------------------
# Prefer an explicit install (MUJOCO_DIR / /opt/mujoco, matching
# mujoco_hw_interface); otherwise fall back to the pip-installed package, whose
# wheel bundles the C library and headers.
if(DEFINED ENV{MUJOCO_DIR})
  set(MUJOCO_DIR $ENV{MUJOCO_DIR})
endif()
if(NOT MUJOCO_DIR)
  execute_process(
    COMMAND python3 -c "import mujoco, os; print(os.path.dirname(mujoco.__file__))"
    OUTPUT_VARIABLE MJ_PKG OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)
endif()

find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h
  PATHS ${MUJOCO_DIR}/include ${MJ_PKG}/include /opt/mujoco/include
  REQUIRED)

# The pip wheel ships a versioned dylib (libmujoco.3.10.0.dylib) that
# find_library's bare "mujoco" name does not match, so glob as a fallback.
find_library(MUJOCO_LIBRARY NAMES mujoco
  PATHS ${MUJOCO_DIR}/lib ${MUJOCO_DIR}/bin ${MJ_PKG} /opt/mujoco/lib)
if(NOT MUJOCO_LIBRARY)
  file(GLOB _mj_glob ${MJ_PKG}/libmujoco*.dylib ${MJ_PKG}/libmujoco*.so
                     ${MUJOCO_DIR}/lib/libmujoco*.dylib ${MUJOCO_DIR}/lib/libmujoco*.so)
  if(_mj_glob)
    list(GET _mj_glob 0 MUJOCO_LIBRARY)
  endif()
endif()
if(NOT MUJOCO_LIBRARY)
  message(FATAL_ERROR "MuJoCo library not found (set MUJOCO_DIR or pip install mujoco)")
endif()
get_filename_component(MUJOCO_LIBDIR "${MUJOCO_LIBRARY}" DIRECTORY)
message(STATUS "MuJoCo include: ${MUJOCO_INCLUDE_DIR}")
message(STATUS "MuJoCo library: ${MUJOCO_LIBRARY}")

set(SCENE_XML "${CMAKE_CURRENT_SOURCE_DIR}/../../kinova_gen2_description/scenes/drawing_demo.xml")

# --- yaml-cpp (libkinematics loads the robot model from YAML) -----------------
find_path(YAMLCPP_INCLUDE_DIR yaml-cpp/yaml.h
  PATHS /opt/homebrew/include /usr/local/include /usr/include REQUIRED)
find_library(YAMLCPP_LIBRARY NAMES yaml-cpp
  PATHS /opt/homebrew/lib /usr/local/lib /usr/lib REQUIRED)

# --- libkinematics: our from-scratch FK/IK/Jacobian/dynamics ------------------
# Compiled straight into the controller so the browser arm runs on our math (no
# ROS/colcon needed for the sim server). Same sources as libkinematics/CMakeLists.
set(LIBKIN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../libkinematics)
set(LIBKIN_SRC
  ${LIBKIN_DIR}/src/math/so3.cpp ${LIBKIN_DIR}/src/math/se3.cpp
  ${LIBKIN_DIR}/src/robot.cpp    ${LIBKIN_DIR}/src/fk.cpp
  ${LIBKIN_DIR}/src/jacobian.cpp ${LIBKIN_DIR}/src/ik.cpp
  ${LIBKIN_DIR}/src/dynamics.cpp ${LIBKIN_DIR}/src/null_space.cpp)
set(CFG ${CMAKE_CURRENT_SOURCE_DIR}/../../kinova_gen2_description/config)

# --- The shared controller (used by the server and the self-check) ------------
add_library(redundancy_controller STATIC
  redundancy_controller.cpp config.cpp ${LIBKIN_SRC})
target_include_directories(redundancy_controller PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR} ${EIGEN3_INCLUDE_DIR} ${MUJOCO_INCLUDE_DIR}
  ${LIBKIN_DIR}/include ${YAMLCPP_INCLUDE_DIR})
target_link_libraries(redundancy_controller PUBLIC ${MUJOCO_LIBRARY} ${YAMLCPP_LIBRARY})
target_compile_definitions(redundancy_controller PUBLIC
  SCENE_XML_PATH="${SCENE_XML}"
  WEBVIZ_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../config.yaml"
  KINOVA_SCREWS_YAML="${CFG}/kinova_gen2_screws.yaml"
  KINOVA_DYNAMICS_YAML="${CFG}/kinova_gen2_dynamics.yaml")

# --- The live WebSocket sim server --------------------------------------------
add_executable(sim_server sim_server.cpp)
target_include_directories(sim_server PRIVATE ${JSON_INCLUDE_DIR})
target_link_libraries(sim_server PRIVATE redundancy_controller)

# --- Self-check ---------------------------------------------------------------
enable_testing()
add_executable(test_controller test_controller.cpp)
target_link_libraries(test_controller PRIVATE redundancy_controller)
add_test(NAME test_controller COMMAND test_controller)

add_executable(test_reach test_reach.cpp)
target_link_libraries(test_reach PRIVATE redundancy_controller)
add_test(NAME test_reach COMMAND test_reach)

# Motion-quality profiler (reporting tool, not a pass/fail test).
add_executable(diag_motion diag_motion.cpp)
target_link_libraries(diag_motion PRIVATE redundancy_controller)

# Runtime path to the (possibly versioned) MuJoCo lib (Linux / explicit install).
set_target_properties(sim_server test_controller test_reach diag_motion PROPERTIES
  BUILD_RPATH "${MUJOCO_LIBDIR}")

# The pip macOS wheel stamps libmujoco with a framework-style install name
# (@rpath/mujoco.framework/Versions/A/...) but ships it as a bare dylib with no
# framework, so dyld can't find it. Repoint each binary's dependency at the real
# file. No-op on an explicit /opt/mujoco install (then the dep already resolves).
if(APPLE)
  foreach(_tgt sim_server test_controller test_reach diag_motion)
    add_custom_command(TARGET ${_tgt} POST_BUILD VERBATIM COMMAND sh -c
      "old=$(otool -L '$<TARGET_FILE:${_tgt}>' | awk '/mujoco/{print $1; exit}'); \
       case \"$old\" in @rpath/*) install_name_tool -change \"$old\" '${MUJOCO_LIBRARY}' '$<TARGET_FILE:${_tgt}>';; esac")
  endforeach()
endif()
