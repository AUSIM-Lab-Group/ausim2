cmake_minimum_required(VERSION 3.20)

project(ausim_msg)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  msg/BoundingBox2D.msg
  msg/BoundingBox2DArray.msg
  msg/BoundingBox3D.msg
  msg/BoundingBox3DArray.msg
  msg/Classification.msg
  msg/Detection2D.msg
  msg/Detection2DArray.msg
  msg/Detection3D.msg
  msg/Detection3DArray.msg
  msg/DeviceCapability.msg
  msg/DeviceStatus.msg
  msg/LabelInfo.msg
  msg/ObjectHypothesis.msg
  msg/ObjectHypothesisWithPose.msg
  msg/Point2D.msg
  msg/Pose2D.msg
  msg/RobotMode.msg
  msg/SimulationEvent.msg
  msg/SimulationEventAck.msg
  msg/VisionClass.msg
  msg/VisionInfo.msg
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)
ament_package()
