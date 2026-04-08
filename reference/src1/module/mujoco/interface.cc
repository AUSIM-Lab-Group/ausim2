
#include <cmath>
#include <cstddef>
#include <random>
#include <vector>

#include "./interface.h"
#include "./model_info.h"
#include "aima/sim/common/db/data_board.h"
#include "aima/sim/data/raycaster.hpp"

// If AIMRT_DEBUG is not available in this translation unit's include path,
// provide a no-op fallback so debug calls compile out and do not print.
#ifndef AIMRT_DEBUG
  #define AIMRT_DEBUG(...) (void)0
#endif

namespace sim::mj {

#define GET_JOINT_STATE(__states__, __type__)                                                   \
  static auto joint_states = aimrte::db::DataBoard().Read<sim::mj::ModelJointStates>(__type__); \
  auto js = joint_states();                                                                     \
  for (auto& s : js) {                                                                          \
    sim::data::Named<sim::data::JointState> state;                                              \
    state.name = s.name;                                                                        \
    if (s.joint_pos_id == 0 && s.joint_vel_id == 0) {                                           \
      state.position = 0.0;                                                                     \
      state.velocity = 0.0;                                                                     \
      state.effort = 0.0;                                                                       \
    } else {                                                                                    \
      state.position = s.position;                                                              \
      state.velocity = s.velocity;                                                              \
      state.effort = s.effort;                                                                  \
    }                                                                                           \
    __states__.joints.push_back(state);                                                         \
  }

#define SET_JOINT_COMMAND(__commands__, __type__)                                                                                               \
  auto joint_commands = aimrte::db::DataBoard().Write<sim::mj::ModelJointCommands>(__type__);                                                   \
  /* 已移除 static 缓存：每次调用都重新建立映射，避免首次初始化后不同关节顺序/数量导致的匹配错误。 */ \
  std::vector<int> cmd_index;                                                                                                                   \
  {                                                                                                                                             \
    std::vector<int> index;                                                                                                                     \
    auto cmd = joint_commands();                                                                                                                \
    index.resize(cmd.size(), -1);                                                                                                               \
    for (std::size_t i = 0; i < cmd.size(); i++) {                                                                                              \
      for (std::size_t j = 0; j < __commands__.joints.size(); j++) {                                                                            \
        if (__commands__.joints[j].name == cmd[i].name) { /* 精确匹配名称 */                                                              \
          index[i] = j;                                                                                                                         \
          break;                                                                                                                                \
        }                                                                                                                                       \
      }                                                                                                                                         \
    }                                                                                                                                           \
    cmd_index = index; /* 每次调用都重新建立映射 */                                                                                  \
  }                                                                                                                                             \
  auto jc = joint_commands();                                                                                                                   \
  for (std::size_t i = 0; i < jc.size(); i++) {                                                                                                 \
    if (cmd_index[i] == -1) continue;                                                                                                           \
    auto& cmd = __commands__.joints[cmd_index[i]];                                                                                              \
    jc[i].position = cmd.position;                                                                                                              \
    jc[i].effort = cmd.effort;                                                                                                                  \
    jc[i].velocity = cmd.velocity;                                                                                                              \
    jc[i].stiffness = cmd.stiffness;                                                                                                            \
    jc[i].damping = cmd.damping;                                                                                                                \
    if (__type__ == sim::mj::JointFrameType::HAND && i < 10) {                                                                                  \
      static int hand_update_count = 0;                                                                                                         \
      if (hand_update_count++ < 30) {                                                                                                           \
        AIMRT_DEBUG("DataBoard Update[{}]: {} <- cmd_joints[{}] pos={} stiff={} damp={}", i, jc[i].name, cmd_index[i], jc[i].position,          \
                    jc[i].stiffness, jc[i].damping);                                                                                            \
      }                                                                                                                                         \
    }                                                                                                                                           \
  }                                                                                                                                             \
  joint_commands = jc;

void GetJointState(sim::data::JointStates& states) { GET_JOINT_STATE(states, sim::mj::JointFrameType::JOINT); }
void GetArmJointState(sim::data::JointStates& states) { GET_JOINT_STATE(states, sim::mj::JointFrameType::ARM); }
void GetLegJointState(sim::data::JointStates& states) { GET_JOINT_STATE(states, sim::mj::JointFrameType::LEG); }
void GetLiftJointState(sim::data::JointStates& states) { GET_JOINT_STATE(states, sim::mj::JointFrameType::LIFT); }
void GetWaistJointState(sim::data::JointStates& states) { GET_JOINT_STATE(states, sim::mj::JointFrameType::WAIST); }
void GetWheelJointState(sim::data::JointStates& states) { GET_JOINT_STATE(states, sim::mj::JointFrameType::WHEEL); }
void GetHeadJointState(sim::data::JointStates& states) { GET_JOINT_STATE(states, sim::mj::JointFrameType::HEAD); }
void GetHandJointState(sim::data::JointStates& states) { GET_JOINT_STATE(states, sim::mj::JointFrameType::HAND); }

void SetJointCommand(sim::data::JointCommands& commands) { SET_JOINT_COMMAND(commands, sim::mj::JointFrameType::JOINT); }
void SetArmJointCommand(sim::data::JointCommands& commands) { SET_JOINT_COMMAND(commands, sim::mj::JointFrameType::ARM); }
void SetLegJointCommand(sim::data::JointCommands& commands) { SET_JOINT_COMMAND(commands, sim::mj::JointFrameType::LEG); }
void SetLiftJointCommand(sim::data::JointCommands& commands) { SET_JOINT_COMMAND(commands, sim::mj::JointFrameType::LIFT); }
void SetWaistJointCommand(sim::data::JointCommands& commands) { SET_JOINT_COMMAND(commands, sim::mj::JointFrameType::WAIST); }
void SetWheelJointCommand(sim::data::JointCommands& commands) { SET_JOINT_COMMAND(commands, sim::mj::JointFrameType::WHEEL); }
void SetHeadJointCommand(sim::data::JointCommands& commands) { SET_JOINT_COMMAND(commands, sim::mj::JointFrameType::HEAD); }
void SetHandJointCommand(sim::data::JointCommands& commands) { SET_JOINT_COMMAND(commands, sim::mj::JointFrameType::HAND); }

void GetImuData(sim::data::ImuData& imu_data, std::string imu_frame_type) {
  const auto imu_frame_type_enum = (imu_frame_type == "imu") ? sim::mj::SensorFrameType::IMU : sim::mj::SensorFrameType::IMU_1;
  auto db_imu = aimrte::db::DataBoard().Read<sim::mj::ModelImuData>(imu_frame_type_enum);
  auto imu = db_imu();
  imu_data.angular_velocity = {imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z};
  imu_data.linear_acceleration = {imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z};
  imu_data.linear_position = {imu.linear_position.x, imu.linear_position.y, imu.linear_position.z};
  imu_data.orientation = {
      imu.orientation.w,
      imu.orientation.x,
      imu.orientation.y,
      imu.orientation.z,
  };
}

void GetOdomData(sim::data::OdomData& odom_data) {
  const auto imu_frame_type_enum = sim::mj::SensorFrameType::ODOM;
  auto db_odom = aimrte::db::DataBoard().Read<sim::mj::ModelOdomData>(imu_frame_type_enum);
  auto odom = db_odom();

  odom_data.position = odom.position;
  odom_data.orientation = odom.orientation;
  odom_data.linear_velocity = odom.linear_velocity;
  odom_data.angular_velocity = odom.angular_velocity;
}
// std::cout << "angular_velocity:" << imu_data.angular_velocity.x << ", " << imu_data.angular_velocity.y << ", " << imu_data.angular_velocity.z <<
// std::endl; std::cout << "linear_position:" << imu_data.linear_position.x << ", " << imu_data.linear_position.y << ", " <<
// imu_data.linear_position.z << std::endl; std::cout << "linear_acceleration:" << imu_data.linear_acceleration.x << ", " <<
// imu_data.linear_acceleration.y << ", " << imu_data.linear_acceleration.z
//           << std::endl;
// std::cout << "orientation:" << imu_data.orientation.x << ", " << imu_data.orientation.y << ", " << imu_data.orientation.z << ", " <<
// imu_data.orientation.w
//           << std::endl;

void GetRaycasterData(sim::data::RaycasterData& raycaster_data, std::string raycaster_name) {
  const auto raycaster_frame_type_enum = sim::mj::SensorFrameType::RAY_CASTER_LIDAR;
  auto db_raycaster = aimrte::db::DataBoard().Read<sim::data::RaycasterData>(raycaster_frame_type_enum);
  auto raycaster = db_raycaster();
  raycaster_data.msg = raycaster.msg;
}

// void GetFootSensorData(std::vector<std::vector<double>>& foot_sensor_data) {
//   static auto foot_sensor = aimrte::db::DataBoard().Read<sim::mj::ModelFootContactData>(sim::mj::SensorFrameType::FOOT_CONTACT);
//   for (auto& data : foot_sensor) {
// }

}  // namespace sim::mj
