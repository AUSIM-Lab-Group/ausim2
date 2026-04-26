cmake_minimum_required(VERSION 3.20)

project(remote_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(yaml-cpp REQUIRED)

add_executable(remote_control_node
  src/main.cpp
  src/remote_control_node.cpp
)

add_executable(remote_control_gui_node
  src/gui_control_model.cpp
  src/gui_runtime_config.cpp
  src/remote_control_gui_main.cpp
)

target_include_directories(remote_control_node
  PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

target_compile_features(remote_control_node PRIVATE cxx_std_17)
target_link_options(remote_control_node PRIVATE "LINKER:--disable-new-dtags")

ament_target_dependencies(
  remote_control_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_srvs
)

target_include_directories(remote_control_gui_node
  PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

target_compile_features(remote_control_gui_node PRIVATE cxx_std_17)
target_link_libraries(remote_control_gui_node Qt5::Widgets yaml-cpp)
target_link_options(remote_control_gui_node PRIVATE "LINKER:--disable-new-dtags")

ament_target_dependencies(
  remote_control_gui_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_srvs
)

install(TARGETS remote_control_node remote_control_gui_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  add_executable(gui_control_model_test
    tests/gui_control_model_test.cpp
    src/gui_control_model.cpp
  )

  target_include_directories(gui_control_model_test
    PRIVATE
      "${CMAKE_CURRENT_SOURCE_DIR}/src"
  )

  target_compile_features(gui_control_model_test PRIVATE cxx_std_17)

  ament_target_dependencies(
    gui_control_model_test
    geometry_msgs
  )

  add_test(NAME gui_control_model_test COMMAND gui_control_model_test)

  add_executable(gui_runtime_config_test
    tests/gui_runtime_config_test.cpp
    src/gui_runtime_config.cpp
  )

  target_include_directories(gui_runtime_config_test
    PRIVATE
      "${CMAKE_CURRENT_SOURCE_DIR}/src"
  )

  target_compile_features(gui_runtime_config_test PRIVATE cxx_std_17)
  target_link_libraries(gui_runtime_config_test yaml-cpp)

  ament_target_dependencies(
    gui_runtime_config_test
    geometry_msgs
  )

  add_test(NAME gui_runtime_config_test COMMAND gui_runtime_config_test)

  add_executable(terminal_keyboard_state_test
    tests/terminal_keyboard_state_test.cpp
    src/remote_control_node.cpp
  )

  target_include_directories(terminal_keyboard_state_test
    PRIVATE
      "${CMAKE_CURRENT_SOURCE_DIR}/src"
  )

  target_compile_features(terminal_keyboard_state_test PRIVATE cxx_std_17)

  ament_target_dependencies(
    terminal_keyboard_state_test
    rclcpp
    geometry_msgs
    sensor_msgs
    std_srvs
  )

  add_test(NAME terminal_keyboard_state_test COMMAND terminal_keyboard_state_test)
endif()

ament_package()
