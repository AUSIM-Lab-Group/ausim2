#pragma once

#include "aima/sim/common/parameter/param_reader.h"
#include "aima/sim/data/imu.hpp"
#include "aima/sim/data/joint.hpp"
#include "aima/sim/data/odom.hpp"
#include "aima/sim/data/raycaster.hpp"

namespace sim::mj {

enum class SensorFrameType { IMU, IMU_1, FOOT_CONTACT, RAY_CASTER_LIDAR, ODOM };
enum class JointControlType { POSITION, VELOCITY, TORQUE };
enum class JointFrameType { JOINT, LEG, ARM, WHEEL, LIFT, WAIST, HEAD, PASSIVE, HAND };

struct ModelInfoActual {
  // actual
  std::vector<std::string> active_joint_frame_type;
  std::vector<std::vector<std::string>> active_joint_name;
  std::vector<std::vector<std::string>> active_joint_control_type;
};
PARAM_REFLECTION(ModelInfoActual, active_joint_frame_type, active_joint_name, active_joint_control_type);

struct ModelInfoSensor {
  // sensor
  std::vector<std::string> passive_joint_frame_type;
  std::vector<std::vector<std::string>> passive_joint_name;
  std::vector<std::string> sensor_frame_type;
  std::vector<std::vector<std::string>> sensor_name;
};
PARAM_REFLECTION(ModelInfoSensor, passive_joint_frame_type, passive_joint_name, sensor_frame_type, sensor_name);

struct ModelInfoLogical {
  // logical
  std::vector<std::vector<double>> nominal_configuration;
};
PARAM_REFLECTION(ModelInfoLogical, nominal_configuration);

struct ModelInfo {
  ModelInfoActual actual;
  ModelInfoSensor sensor;
  ModelInfoLogical logical;
};
PARAM_REFLECTION(ModelInfo, actual, sensor, logical);

struct ModelJointCommand : public ::sim::data::JointCommand {
  std::string name;
  int actuator_id;
  JointControlType control_type;
};

struct ModelJointState : public ::sim::data::JointState {
  std::string name;
  int joint_pos_id{0};
  int joint_vel_id{0};
};

struct ModelImuData : public ::sim::data::ImuData {
  int orientation_id{0};
  int angular_vel_id{0};
  int linear_pos_id{0};
  int linear_vel_id{0};
  int linear_acc_id{0};
};

struct ModelRaycasterData : public ::sim::data::RaycasterData {
  sensor_msgs::msg::PointCloud2 msg;
};

struct ModelOdomData : public ::sim::data::OdomData {
  std::string name = "pelvis";  // 基座名称
  int body_id = -1;             // 基座body ID（Mujoco中的ID）
};

struct ModelFootContactData {
  int contact_id{0};
  std::string name;
  double foot_contact{0};
};

using ModelImuDatas = std::vector<ModelImuData>;
using ModelJointStates = std::vector<ModelJointState>;
using ModelJointCommands = std::vector<ModelJointCommand>;
using ModelFootContactDatas = std::vector<ModelFootContactData>;
// using ModelRaycasterDatas = std::vector<ModelRaycasterData>;
}  // namespace sim::mj
