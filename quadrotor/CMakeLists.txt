cmake_minimum_required(VERSION 3.20)

project(quadrotor_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(glfw3 REQUIRED)

foreach(_quadrotor_ros_prefix "/opt/ros/humble" "/opt/ros/humble_py311")
  if(EXISTS "${_quadrotor_ros_prefix}")
    list(APPEND CMAKE_PREFIX_PATH "${_quadrotor_ros_prefix}")
    if(DEFINED ENV{AMENT_PREFIX_PATH} AND NOT "$ENV{AMENT_PREFIX_PATH}" STREQUAL "")
      set(ENV{AMENT_PREFIX_PATH} "${_quadrotor_ros_prefix}:$ENV{AMENT_PREFIX_PATH}")
    else()
      set(ENV{AMENT_PREFIX_PATH} "${_quadrotor_ros_prefix}")
    endif()
  endif()
endforeach()

foreach(_quadrotor_ros_python "/opt/ros/humble/lib/python3.10/site-packages" "/opt/ros/humble_py311/lib/python3.11/site-packages")
  if(EXISTS "${_quadrotor_ros_python}")
    if(DEFINED ENV{PYTHONPATH} AND NOT "$ENV{PYTHONPATH}" STREQUAL "")
      set(ENV{PYTHONPATH} "${_quadrotor_ros_python}:$ENV{PYTHONPATH}")
    else()
      set(ENV{PYTHONPATH} "${_quadrotor_ros_python}")
    endif()
  endif()
endforeach()

find_package(mujoco CONFIG QUIET)
if(NOT mujoco_FOUND)
  set(_quadrotor_mujoco_hints)
  if(DEFINED mujoco_DIR)
    list(APPEND _quadrotor_mujoco_hints "${mujoco_DIR}")
  endif()
  if(DEFINED ENV{MUJOCO_DIR})
    list(APPEND _quadrotor_mujoco_hints "$ENV{MUJOCO_DIR}")
  endif()
  if(DEFINED ENV{MUJOCO_ROOT})
    list(APPEND _quadrotor_mujoco_hints "$ENV{MUJOCO_ROOT}" "$ENV{MUJOCO_ROOT}/build")
  endif()
  list(APPEND _quadrotor_mujoco_hints
       "/home/x/mujoco/mujoco-3.3.7/build"
       "/usr/local/lib/cmake/mujoco"
       "/usr/lib/cmake/mujoco")
  find_package(mujoco CONFIG REQUIRED PATHS ${_quadrotor_mujoco_hints})
endif()

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
    mujoco::mujoco
    "${QUADROTOR_THIRD_PARTY_DIR}/mujoco_simulate/liblodepng.a"
)

add_library(quadrotor_sim_core STATIC
  src/app/quadrotor_app.cpp
  src/config/quadrotor_config.cpp
  src/converts/ipc/bridge_packets.cpp
  src/control/motor_mixer.cpp
  src/controller/se3_controller.cpp
  src/manager/ros_bridge_process_manager.cpp
  src/math/geometry.cpp
  src/runtime/data_board_interface.cpp
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
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

target_compile_features(quadrotor_sim_core PUBLIC cxx_std_17)
target_compile_definitions(quadrotor_sim_core
  PUBLIC
    QUADROTOR_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

add_library(quadrotor_ros_bridge_core STATIC
  src/config/quadrotor_config.cpp
  src/converts/data/clock.cpp
  src/converts/data/cmd_vel.cpp
  src/converts/data/common.cpp
  src/converts/data/image.cpp
  src/converts/data/imu.cpp
  src/converts/data/odom.cpp
  src/converts/data/transform.cpp
  src/converts/ipc/bridge_packets.cpp
  src/ros/publisher/data/clock_data_publisher.cpp
  src/ros/publisher/data/image_data_publisher.cpp
  src/ros/publisher/data/imu_data_publisher.cpp
  src/ros/publisher/data/odom_data_publisher.cpp
  src/ros/publisher/data/transform_data_publisher.cpp
  src/ros/ros2_bridge.cpp
  src/ros/subscriber/data/cmd_vel_command_subscriber.cpp
)

target_include_directories(quadrotor_ros_bridge_core
  PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

target_compile_features(quadrotor_ros_bridge_core PUBLIC cxx_std_17)
target_compile_definitions(quadrotor_ros_bridge_core
  PUBLIC
    QUADROTOR_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

ament_target_dependencies(
  quadrotor_ros_bridge_core
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  rosgraph_msgs
  tf2
  tf2_ros
)

target_link_libraries(quadrotor_sim_core
  Eigen3::Eigen
  glfw
  yaml-cpp
  quadrotor_mujoco_viewer
  mujoco::mujoco
)

target_link_libraries(quadrotor_ros_bridge_core
  Eigen3::Eigen
  yaml-cpp
)

add_executable(quadrotor
  src/main.cpp
)

target_link_options(quadrotor PRIVATE "LINKER:--disable-new-dtags")
target_link_libraries(quadrotor PRIVATE quadrotor_sim_core)

add_executable(quadrotor_ros_bridge
  src/ros/ros_bridge_main.cpp
)

target_link_options(quadrotor_ros_bridge PRIVATE "LINKER:--disable-new-dtags")
target_link_libraries(quadrotor_ros_bridge PRIVATE quadrotor_ros_bridge_core)

get_target_property(_quadrotor_mujoco_lib mujoco::mujoco IMPORTED_LOCATION_RELEASE)
if(NOT _quadrotor_mujoco_lib)
  get_target_property(_quadrotor_mujoco_lib mujoco::mujoco IMPORTED_LOCATION_RELWITHDEBINFO)
endif()
if(NOT _quadrotor_mujoco_lib)
  get_target_property(_quadrotor_mujoco_lib mujoco::mujoco IMPORTED_LOCATION_DEBUG)
endif()
if(NOT _quadrotor_mujoco_lib)
  get_target_property(_quadrotor_mujoco_lib mujoco::mujoco IMPORTED_LOCATION)
endif()
set(_quadrotor_runtime_rpaths)
set(_quadrotor_ros_runtime_rpaths)
if(_quadrotor_mujoco_lib)
  get_filename_component(_quadrotor_mujoco_dir "${_quadrotor_mujoco_lib}" DIRECTORY)
  list(APPEND _quadrotor_runtime_rpaths "${_quadrotor_mujoco_dir}")
endif()
foreach(_quadrotor_ros_libdir "/opt/ros/humble/lib" "/opt/ros/humble_py311/lib")
  if(EXISTS "${_quadrotor_ros_libdir}")
    list(APPEND _quadrotor_ros_runtime_rpaths "${_quadrotor_ros_libdir}")
  endif()
endforeach()
if(_quadrotor_runtime_rpaths)
  set_target_properties(quadrotor PROPERTIES
    BUILD_RPATH "${_quadrotor_runtime_rpaths}"
    INSTALL_RPATH "${_quadrotor_runtime_rpaths}"
  )
endif()
if(_quadrotor_ros_runtime_rpaths)
  set_target_properties(quadrotor_ros_bridge PROPERTIES
    BUILD_RPATH "${_quadrotor_ros_runtime_rpaths}"
    INSTALL_RPATH "${_quadrotor_ros_runtime_rpaths}"
  )
endif()

install(TARGETS quadrotor quadrotor_ros_bridge quadrotor_sim_core quadrotor_ros_bridge_core quadrotor_mujoco_viewer
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(DIRECTORY "${CMAKE_SOURCE_DIR}/assets" DESTINATION share)
install(FILES README.md DESTINATION share/quadrotor)
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/cfg" DESTINATION share/quadrotor)
