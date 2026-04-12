cmake_minimum_required(VERSION 3.20)

project(quadrotor_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT DEFINED MUJOCO_IMPORTED_TARGET)
  message(FATAL_ERROR "MuJoCo dependency was not resolved before entering quadrotor/.")
endif()

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(glfw3 REQUIRED)

set(QUADROTOR_THIRD_PARTY_DIR "${CMAKE_SOURCE_DIR}/third_party")

add_library(quadrotor_mujoco_viewer STATIC
  "${QUADROTOR_THIRD_PARTY_DIR}/mujoco_simulate/simulate.cc"
  "${QUADROTOR_THIRD_PARTY_DIR}/mujoco_simulate/platform_ui_adapter.cc"
  "${QUADROTOR_THIRD_PARTY_DIR}/mujoco_simulate/glfw_adapter.cc"
  "${QUADROTOR_THIRD_PARTY_DIR}/mujoco_simulate/glfw_dispatch.cc"
)

target_include_directories(quadrotor_mujoco_viewer
  PUBLIC
    "${QUADROTOR_THIRD_PARTY_DIR}"
)

target_compile_features(quadrotor_mujoco_viewer PUBLIC cxx_std_17)

target_link_libraries(quadrotor_mujoco_viewer
  PUBLIC
    glfw
    "${MUJOCO_IMPORTED_TARGET}"
    "${QUADROTOR_THIRD_PARTY_DIR}/mujoco_simulate/liblodepng.a"
)

add_subdirectory("${QUADROTOR_THIRD_PARTY_DIR}/dynamic_obs_generator" "dynamic_obs_generator_build")

add_library(quadrotor_sim_core STATIC
  src/app/quadrotor_app.cpp
  src/control/motor_mixer.cpp
  src/controller/se3_controller.cpp
  src/math/geometry.cpp
  src/runtime/goal_provider.cpp
  src/runtime/vehicle_runtime.cpp
  src/sim/camera_renderer.cpp
  src/sim/mujoco_actuator_writer.cpp
  src/sim/mujoco_bindings.cpp
  src/sim/mujoco_state_reader.cpp
  src/sim/quadrotor_sim.cpp
)

target_include_directories(quadrotor_sim_core
  PUBLIC
    "${CMAKE_SOURCE_DIR}/ausim_common/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

target_compile_features(quadrotor_sim_core PUBLIC cxx_std_17)
target_compile_definitions(quadrotor_sim_core
  PUBLIC
    QUADROTOR_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(quadrotor_sim_core
  Eigen3::Eigen
  glfw
  yaml-cpp
  ausim_common_runtime
  quadrotor_mujoco_viewer
  dynamic_obs_generator
  "${MUJOCO_IMPORTED_TARGET}"
)

add_executable(quadrotor
  src/main.cpp
)

target_link_options(quadrotor PRIVATE "LINKER:--disable-new-dtags")
target_link_libraries(quadrotor PRIVATE quadrotor_sim_core)

set(_quadrotor_runtime_rpaths)
if(MUJOCO_RESOLVED_LIBRARY_DIR)
  list(APPEND _quadrotor_runtime_rpaths "${MUJOCO_RESOLVED_LIBRARY_DIR}")
endif()
if(_quadrotor_runtime_rpaths)
  set_target_properties(quadrotor PROPERTIES
    BUILD_RPATH "${_quadrotor_runtime_rpaths}"
    INSTALL_RPATH "${_quadrotor_runtime_rpaths}"
  )
endif()
if(MUJOCO_RESOLVED_PLUGIN_DIR)
  target_compile_definitions(quadrotor_sim_core
    PUBLIC
      QUADROTOR_MUJOCO_PLUGIN_FALLBACK_DIR="${MUJOCO_RESOLVED_PLUGIN_DIR}"
  )
endif()

install(TARGETS quadrotor quadrotor_sim_core quadrotor_mujoco_viewer
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(DIRECTORY "${CMAKE_SOURCE_DIR}/assets" DESTINATION share)
install(FILES README.md DESTINATION share/quadrotor)
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/cfg" DESTINATION share/quadrotor)
