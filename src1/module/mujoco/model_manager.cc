#include "./model_manager.h"

#include <array_safety.h>
#include <mujoco/mujoco.h>
#include <cstddef>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "./model_info.h"
#include "aima/sim/common/db/data_board.h"
#include "aima/sim/common/parameter/param_util.h"
#include "aima/sim/data/raycaster.hpp"
#include "util/log_util.h"

namespace sim::mj {
auto GetLogger() { return aimrt::common::util::SimpleLogger(); }
// 存储 Raycaster 信息
int data_pos = 0;
int pos_w_data_point = 0;
int pos_w_data_size = 0;
int pos_b_data_point = 0;
int pos_b_data_size = 0;
int camera_quat_sensor_d_point = 0;
int camera_pos_sensor_d_point = 0;
std::tuple<int, int, std::vector<std::pair<int, int>>> get_ray_caster_info(const mjModel* model_, mjData* d, const std::string& sensor_name) {
  std::vector<std::pair<int, int>> data_ps;
  int sensor_id = mj_name2id(model_, mjOBJ_SENSOR, sensor_name.c_str());
  if (sensor_id == -1) {
    AIMRT_ERROR("Raycaster sensor not found");
    return std::make_tuple(0, 0, data_ps);
  }
  int sensor_plugin_id = model_->sensor_plugin[sensor_id];
  int state_idx = model_->plugin_stateadr[sensor_plugin_id];

  for (int i = state_idx + 2; i < state_idx + model_->plugin_statenum[sensor_plugin_id]; i += 2) {
    data_ps.emplace_back(d->plugin_state[i], d->plugin_state[i + 1]);
  }
  int h_ray_num = d->plugin_state[state_idx + 0];
  int v_ray_num = d->plugin_state[state_idx + 1];
  return std::make_tuple(h_ray_num, v_ray_num, data_ps);
}

static sim::mj::SensorFrameType GetSensorFrameType(const std::string& frame_type) {
  static const std::map<std::string, sim::mj::SensorFrameType> frame_type_map{{"imu", sim::mj::SensorFrameType::IMU},
                                                                              {"imu_1", sim::mj::SensorFrameType::IMU_1},
                                                                              {"foot_contact", sim::mj::SensorFrameType::FOOT_CONTACT},
                                                                              {"raycaster_lidar", sim::mj::SensorFrameType::RAY_CASTER_LIDAR},
                                                                              {"odom", sim::mj::SensorFrameType::ODOM}};
  return frame_type_map.at(frame_type);
}

static sim::mj::JointFrameType GetFrameType(const std::string& frame_type) {
  static const std::map<std::string, sim::mj::JointFrameType> frame_type_map{
      {"joint", sim::mj::JointFrameType::JOINT}, {"leg", sim::mj::JointFrameType::LEG},   {"arm", sim::mj::JointFrameType::ARM},
      {"wheel", sim::mj::JointFrameType::WHEEL}, {"lift", sim::mj::JointFrameType::LIFT}, {"waist", sim::mj::JointFrameType::WAIST},
      {"head", sim::mj::JointFrameType::HEAD},   {"hand", sim::mj::JointFrameType::HAND},
  };
  return frame_type_map.at(frame_type);
}

static sim::mj::JointControlType GetControlType(const std::string& control_type) {
  static const std::map<std::string, sim::mj::JointControlType> control_type_map{
      {"position", sim::mj::JointControlType::POSITION},
      {"velocity", sim::mj::JointControlType::VELOCITY},
      {"torque", sim::mj::JointControlType::TORQUE},
  };
  return control_type_map.at(control_type);
}

ModelManager::ModelManager(std::shared_ptr<mujoco::Simulate> sim, const std::string& model_info) : model_info_(model_info), sim_(sim) {}

ModelManager::~ModelManager() { Reset(); }

void ModelManager::Reset() {
  if (data_) {
    const std::lock_guard<std::mutex> lock(data_mutex_);
    mj_deleteData(data_);
  }
  if (model_) {
    mj_deleteModel(model_);
  }
}

void ModelManager::InitModel() {
  static bool is_init = false;
  if (is_init) return;
  is_init = true;

  sim::mj::ModelInfo model_info;
  param::ReadParam(model_info, model_info_);

  auto GetJointStates = [this](std::vector<ModelJointState>& s, const std::vector<std::string>& joint_names) {
    for (auto& joint_name : joint_names) {
      sim::mj::ModelJointState joint_state;
      joint_state.name = joint_name;
      auto joint_pos = "jointpos_" + joint_name;
      auto joint_vel = "jointvel_" + joint_name;
      joint_state.joint_pos_id = model_->sensor_adr[mj_name2id(model_, mjOBJ_SENSOR, joint_pos.c_str())];
      joint_state.joint_vel_id = model_->sensor_adr[mj_name2id(model_, mjOBJ_SENSOR, joint_vel.c_str())];
      if (joint_state.joint_pos_id == 0 && joint_state.joint_vel_id == 0) {
        AIMRT_INFO("Can not find joint state: {}", joint_name);
        // continue;
      }

      s.push_back(joint_state);
    }
  };

  for (std::size_t i = 0; i < model_info.actual.active_joint_frame_type.size(); i++) {
    const auto& frame_type = model_info.actual.active_joint_frame_type[i];

    if (frame_type.empty()) continue;

    joint_frame_types_.insert(GetFrameType(frame_type));

    const std::vector<std::string>& joint_names = model_info.actual.active_joint_name[i];
    const std::vector<std::string>& joint_control_type = model_info.actual.active_joint_control_type[i];
    const std::vector<double>& joint_nominal =
        (model_info.logical.nominal_configuration.size() > i) ? model_info.logical.nominal_configuration[i] : std::vector<double>();

    // 校验
    if (joint_names.size() != joint_control_type.size()) {
      throw std::runtime_error(frame_type + ": joint names and joint control type size not equal ...");
    }

    // 数据黑板存储 joint 状态信息
    auto joint_states = aimrte::db::DataBoard().Write<sim::mj::ModelJointStates>(GetFrameType(frame_type));
    joint_states.LockForWrite([&](auto& s) { GetJointStates(s, joint_names); });

    // 数据黑板存储 joint 命令信息
    auto joint_commands = aimrte::db::DataBoard().Write<sim::mj::ModelJointCommands>(GetFrameType(frame_type));
    joint_commands.LockForWrite([&](auto& c) {
      sim::mj::ModelJointCommand joint_command;
      for (std::size_t j = 0; j < joint_names.size(); j++) {
        const std::string& joint_name = joint_names[j];
        // joint_command.position = joint_nominal[j];
        joint_command.control_type = GetControlType(joint_control_type[j]);
        if (joint_nominal.size() > j) {
          if (joint_command.control_type == sim::mj::JointControlType::POSITION) {
            joint_command.position = joint_nominal[j];
          } else if (joint_command.control_type == sim::mj::JointControlType::VELOCITY) {
            joint_command.velocity = joint_nominal[j];
          } else if (joint_command.control_type == sim::mj::JointControlType::TORQUE) {
            joint_command.effort = joint_nominal[j];
          }
        }

        joint_command.name = joint_name;
        std::string actuator_name = "motor_" + joint_name;
        joint_command.actuator_id = mj_name2id(model_, mjOBJ_ACTUATOR, actuator_name.c_str());

        // fallback 1: 去掉 _joint 后缀
        std::string short_name = joint_name;
        if (joint_command.actuator_id == -1 && joint_name.size() > 6 && joint_name.substr(joint_name.size() - 6) == "_joint") {
          short_name = joint_name.substr(0, joint_name.size() - 6);
          actuator_name = "motor_" + short_name;
          joint_command.actuator_id = mj_name2id(model_, mjOBJ_ACTUATOR, actuator_name.c_str());
        }
        // fallback 2: motor_<short>_act
        if (joint_command.actuator_id == -1) {
          actuator_name = "motor_" + short_name + "_act";
          joint_command.actuator_id = mj_name2id(model_, mjOBJ_ACTUATOR, actuator_name.c_str());
        }
        // fallback 3: motor_<short>_motor
        if (joint_command.actuator_id == -1) {
          actuator_name = "motor_" + short_name + "_motor";
          joint_command.actuator_id = mj_name2id(model_, mjOBJ_ACTUATOR, actuator_name.c_str());
        }

        c.push_back(joint_command);
      }
    });
  }

  // 被动关节信息
  for (std::size_t i = 0; i < model_info.sensor.passive_joint_frame_type.size(); i++) {
    const auto& frame_type = model_info.sensor.passive_joint_frame_type[i];
    if (frame_type.empty()) continue;

    const std::vector<std::string>& joint_names = model_info.sensor.passive_joint_name[i];

    // 数据黑板存储 joint 状态信息
    auto joint_states = aimrte::db::DataBoard().Write<sim::mj::ModelJointStates>(GetFrameType(frame_type));
    joint_states.LockForWrite([&](auto& s) { GetJointStates(s, joint_names); });
  }

  // 存储 IMU 信息
  for (std::size_t i = 0; i < model_info.sensor.sensor_frame_type.size(); i++) {
    const auto& frame_type = model_info.sensor.sensor_frame_type[i];
    if (frame_type.empty()) continue;

    sensor_frame_types_.insert(GetSensorFrameType(frame_type));

    auto sensor_frame = GetSensorFrameType(frame_type);
    auto& sensor_names = model_info.sensor.sensor_name[i];

    switch (sensor_frame) {
      case sim::mj::SensorFrameType::IMU:
      case sim::mj::SensorFrameType::IMU_1: {
        sim::mj::ModelImuData model_imu_data{};
        auto imu_data = aimrte::db::DataBoard().Write<sim::mj::ModelImuData>(sensor_frame);
        for (std::size_t j = 0; j < sensor_names.size(); j++) {
          auto& sensor_name = sensor_names[j];
          if (sensor_name.find("orientation") != std::string::npos) {
            model_imu_data.orientation_id = model_->sensor_adr[mj_name2id(model_, mjOBJ_SENSOR, sensor_name.c_str())];
          } else if ((sensor_name.find("angular") != std::string::npos) && (sensor_name.find("vel") != std::string::npos)) {
            model_imu_data.angular_vel_id = model_->sensor_adr[mj_name2id(model_, mjOBJ_SENSOR, sensor_name.c_str())];
          } else if ((sensor_name.find("linear") != std::string::npos && (sensor_name.find("pos") != std::string::npos))) {
            model_imu_data.linear_pos_id = model_->sensor_adr[mj_name2id(model_, mjOBJ_SENSOR, sensor_name.c_str())];
          } else if ((sensor_name.find("linear") != std::string::npos) && (sensor_name.find("vel") != std::string::npos)) {
            model_imu_data.linear_vel_id = model_->sensor_adr[mj_name2id(model_, mjOBJ_SENSOR, sensor_name.c_str())];
          } else if ((sensor_name.find("linear") != std::string::npos) && (sensor_name.find("acc") != std::string::npos)) {
            model_imu_data.linear_acc_id = model_->sensor_adr[mj_name2id(model_, mjOBJ_SENSOR, sensor_name.c_str())];
          }
        }
        imu_data = model_imu_data;
        break;
      }
      case sim::mj::SensorFrameType::FOOT_CONTACT: {
        auto foot_contacts = aimrte::db::DataBoard().Write<sim::mj::ModelFootContactDatas>(sensor_frame);
        foot_contacts.LockForWrite([&](auto& contacts) {
          for (auto& sensor_name : sensor_names) {
            sim::mj::ModelFootContactData data;
            data.contact_id = model_->sensor_adr[mj_name2id(model_, mjOBJ_SENSOR, sensor_name.c_str())];
            data.name = sensor_name;
            contacts.push_back(data);
          }
        });
        break;
      }
      case sim::mj::SensorFrameType::ODOM: {
        sim::mj::ModelOdomData odom_data;
        odom_data.name = sensor_names[0];  // 从sensor_name获取基座名称
        odom_data.body_id = mj_name2id(model_, mjOBJ_BODY, odom_data.name.c_str());
        if (odom_data.body_id < 0) {
          AIMRT_ERROR("Odom base body '{}' not found", odom_data.name);
          break;
        }
        // 写入数据黑板
        auto db_odom = aimrte::db::DataBoard().Write<sim::mj::ModelOdomData>(sensor_frame);
        db_odom = odom_data;
        AIMRT_INFO("Odom initialized: body_id={}, name={}", odom_data.body_id, odom_data.name);
        break;
      }
      default:
        break;
    }
  }

  // 存储raycaster_lidar数据（若不存在则标记为不可用）
  has_raycaster_ = false;
  int sensor_id = mj_name2id(model_, mjOBJ_SENSOR, "raycaster_lidar");
  if (sensor_id != -1) {
    data_pos = model_->sensor_adr[sensor_id];

    int quat_id = mj_name2id(model_, mjOBJ_SENSOR, "camera_quat");
    int pos_id = mj_name2id(model_, mjOBJ_SENSOR, "camera_pos");
    if (quat_id != -1) {
      camera_quat_sensor_d_point = model_->sensor_adr[quat_id];
    }
    if (pos_id != -1) {
      camera_pos_sensor_d_point = model_->sensor_adr[pos_id];
    }

    auto [h_ray_num, v_ray_num, data_pairs] = sim::mj::get_ray_caster_info(model_, data_, "raycaster_lidar");
    AIMRT_DEBUG("h_ray_num: {}, v_ray_num: {}", h_ray_num, v_ray_num);
    AIMRT_DEBUG("data_ps: ");
    for (const auto& pair : data_pairs) {
      AIMRT_DEBUG("({}, {})", pair.first, pair.second);
    }

    if (data_pairs.size() >= 2) {
      pos_w_data_point = data_pairs[0].first;
      pos_w_data_size = data_pairs[0].second;
      pos_b_data_point = data_pairs[1].first;
      pos_b_data_size = data_pairs[1].second;
      has_raycaster_ = true;
    } else {
      AIMRT_ERROR("raycaster_lidar data_pairs insufficient, disable raycaster publishing");
    }
  } else {
    AIMRT_ERROR("raycaster_lidar not found, disable raycaster publishing");
  }
}

// std::cout << aimrte::db::DataBoard().Info() << std::endl;

bool ModelManager::LoadModel(const std::string& simulation_model) {
  if (!sim_) return false;
  mjData* dnew = nullptr;
  mjModel* mnew = nullptr;

  sim_->LoadMessage(simulation_model.c_str());

  // 检查文件名是否以 .mjb 结尾，如果是，则调用 mj_loadModel 加载二进制模型文件。
  if (simulation_model.size() > 4 &&
      !std::strncmp(simulation_model.c_str() + simulation_model.size() - 4, ".mjb", simulation_model.size() - simulation_model.size() + 4)) {
    mnew = mj_loadModel(simulation_model.c_str(), nullptr);
  } else {
    mnew = mj_loadXML(simulation_model.c_str(), nullptr, nullptr, 0);
  }

  if (mnew) dnew = mj_makeData(mnew);

  if (dnew) {
    sim_->Load(mnew, dnew, simulation_model.c_str());
    Reset();
    data_ = dnew;
    model_ = mnew;
  } else {
    sim_->LoadMessageClear();
    return false;
  }

  InitModel();

  // 在模型加载后设置初始qpos为nominal_configuration的值，确保初始姿态正确（特别是手腕pitch）
  if (data_ && model_) {
    sim::mj::ModelInfo model_info;
    param::ReadParam(model_info, model_info_);

    AIMRT_INFO("========== Setting initial qpos from nominal_configuration ==========");

    for (std::size_t i = 0; i < model_info.actual.active_joint_frame_type.size(); i++) {
      const auto& frame_type = model_info.actual.active_joint_frame_type[i];
      if (frame_type.empty()) continue;

      const std::vector<std::string>& joint_names = model_info.actual.active_joint_name[i];
      const std::vector<double>& joint_nominal =
          (model_info.logical.nominal_configuration.size() > i) ? model_info.logical.nominal_configuration[i] : std::vector<double>();

      if (joint_nominal.size() != joint_names.size()) {
        char warn_buf[256];
        snprintf(warn_buf, sizeof(warn_buf), "WARNING: Frame %s nominal size (%zu) != joint names size (%zu)", frame_type.c_str(),
                 joint_nominal.size(), joint_names.size());
        continue;
      }

      for (std::size_t j = 0; j < joint_names.size(); j++) {
        const std::string& joint_name = joint_names[j];
        int joint_id = mj_name2id(model_, mjOBJ_JOINT, joint_name.c_str());
        if (joint_id >= 0 && joint_id < model_->njnt) {
          int qpos_id = model_->jnt_qposadr[joint_id];
          if (qpos_id >= 0 && qpos_id < model_->nq) {
            data_->qpos[qpos_id] = joint_nominal[j];
          }
        }
      }
    }

    // 重置数据，应用qpos设置
    mj_forward(model_, data_);

    AIMRT_INFO("========== Initial qpos setting complete ==========");
  }

  return (data_ != nullptr);
}

void ModelManager::UpdateModelState() {
  if (model_ == nullptr || data_ == nullptr) return;

  for (auto& joint_frame_type : joint_frame_types_) {
    auto db_joint_states = aimrte::db::DataBoard().Write<sim::mj::ModelJointStates>(joint_frame_type);
    auto joint_states = db_joint_states();
    for (ModelJointState& state : joint_states) {
      state.position = data_->sensordata[state.joint_pos_id];
      state.velocity = data_->sensordata[state.joint_vel_id];
    }
    db_joint_states = joint_states;
  }

  for (auto& sensor_frame_type : sensor_frame_types_) {
    switch (sensor_frame_type) {
      case sim::mj::SensorFrameType::IMU:
      case sim::mj::SensorFrameType::IMU_1: {
        auto imu_data = aimrte::db::DataBoard().Write<sim::mj::ModelImuData>(sensor_frame_type);
        auto data = imu_data();

        data.angular_velocity = {data_->sensordata[data.angular_vel_id], data_->sensordata[data.angular_vel_id + 1],
                                 data_->sensordata[data.angular_vel_id + 2]};
        data.linear_position = {data_->sensordata[data.linear_pos_id], data_->sensordata[data.linear_pos_id + 1],
                                data_->sensordata[data.linear_pos_id + 2]};
        data.linear_acceleration = {data_->sensordata[data.linear_acc_id], data_->sensordata[data.linear_acc_id + 1],
                                    data_->sensordata[data.linear_acc_id + 2]};
        data.orientation = {data_->sensordata[data.orientation_id], data_->sensordata[data.orientation_id + 1],
                            data_->sensordata[data.orientation_id + 2], data_->sensordata[data.orientation_id + 3]};
        imu_data = data;
        break;
      }
      case sim::mj::SensorFrameType::FOOT_CONTACT: {
        auto foot_contacts = aimrte::db::DataBoard().Write<sim::mj::ModelFootContactDatas>(sensor_frame_type);
        foot_contacts.LockForWrite([&](ModelFootContactDatas& contacts) {
          for (ModelFootContactData& contact : contacts) {
            contact.foot_contact = data_->sensordata[contact.contact_id];
          }
        });
        break;
      }
      case sim::mj::SensorFrameType::ODOM: {
        auto db_odom = aimrte::db::DataBoard().Write<sim::mj::ModelOdomData>(sensor_frame_type);
        auto odom_data = db_odom();  // 读取当前数据（包含body_id）

        if (odom_data.body_id < 0) {
          continue;  // 未初始化则跳过
        }

        odom_data.position.x = data_->xpos[3 * odom_data.body_id + 0];  // x
        odom_data.position.y = data_->xpos[3 * odom_data.body_id + 1];  // y
        odom_data.position.z = data_->xpos[3 * odom_data.body_id + 2];  // z

        odom_data.orientation.w = data_->xquat[4 * odom_data.body_id + 0];  // w
        odom_data.orientation.x = data_->xquat[4 * odom_data.body_id + 1];  // x
        odom_data.orientation.y = data_->xquat[4 * odom_data.body_id + 2];  // y
        odom_data.orientation.z = data_->xquat[4 * odom_data.body_id + 3];  // z

        // 线速度
        odom_data.linear_velocity.x = data_->cvel[6 * odom_data.body_id + 3];
        odom_data.linear_velocity.y = data_->cvel[6 * odom_data.body_id + 4];
        odom_data.linear_velocity.z = data_->cvel[6 * odom_data.body_id + 5];

        // 角速度
        odom_data.angular_velocity.x = data_->cvel[6 * odom_data.body_id + 0];
        odom_data.angular_velocity.y = data_->cvel[6 * odom_data.body_id + 1];
        odom_data.angular_velocity.z = data_->cvel[6 * odom_data.body_id + 2];

        // 写回数据黑板
        db_odom = odom_data;
      }
      default:
        break;
    }
  }

  // raycaster data（若不可用则发布空消息）
  auto ray_caster_data = aimrte::db::DataBoard().Write<sim::data::RaycasterData>(sim::mj::SensorFrameType::RAY_CASTER_LIDAR);
  if (!has_raycaster_) {
    sensor_msgs::msg::PointCloud2 empty;
    empty.header.stamp = rclcpp::Clock().now();
    empty.header.frame_id = "map";
    empty.height = 1;
    empty.width = 0;
    empty.point_step = 12;
    empty.row_step = 0;
    empty.is_dense = false;
    ray_caster_data.LockForWrite([&](sim::data::RaycasterData& d) { d.msg = empty; });
    return;
  }

  // Pointcloud_Pub
  mjtNum* point_cloud_data = data_->sensordata + data_pos + pos_w_data_point;
  int len = pos_w_data_size / 3;  // TODO: 使用实际长度（目前为占位值）
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.stamp = rclcpp::Clock().now();
  msg.header.frame_id = "map";
  msg.fields.resize(3);
  msg.fields[0].name = "x";
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[0].count = 1;
  msg.fields[1].name = "y";
  msg.fields[1].offset = 4;
  msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[1].count = 1;
  msg.fields[2].name = "z";
  msg.fields[2].offset = 8;
  msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[2].count = 1;
  msg.height = 1;
  msg.width = len;      // 点的数量
  msg.point_step = 12;  // 每个点的大小（3个float * 4字节）
  msg.row_step = msg.point_step * msg.width;
  msg.is_dense = false;
  msg.data.resize(msg.row_step);
  float* data_ptr = reinterpret_cast<float*>(msg.data.data());
  for (int i = 0; i < len; i++) {
    data_ptr[0] = static_cast<float>(point_cloud_data[i * 3 + 0]);  // x
    data_ptr[1] = static_cast<float>(point_cloud_data[i * 3 + 1]);  // y
    data_ptr[2] = static_cast<float>(point_cloud_data[i * 3 + 2]);  // z
    data_ptr += 3;
  }

  ray_caster_data.LockForWrite([&](sim::data::RaycasterData& d) { d.msg = msg; });
}

void ModelManager::UpdateModelCommand() {
  if (model_ == nullptr || data_ == nullptr) return;

  for (auto& joint_frame_type : joint_frame_types_) {
    sim::mj::ModelJointCommands joint_commands = aimrte::db::DataBoard().Read<sim::mj::ModelJointCommands>(joint_frame_type)();
    sim::mj::ModelJointStates joint_states = aimrte::db::DataBoard().Read<sim::mj::ModelJointStates>(joint_frame_type)();

    for (std::size_t i = 0; i < joint_commands.size(); i++) {
      ModelJointCommand& command = joint_commands[i];
      double position = command.position;  // default
      if (command.control_type == sim::mj::JointControlType::VELOCITY) {
        position = command.velocity;
      } else if (command.control_type == sim::mj::JointControlType::TORQUE) {
        ModelJointState& state = joint_states[i];
        position = command.effort + command.stiffness * (command.position - state.position) + command.damping * (command.velocity - state.velocity);
      } else if (command.control_type == sim::mj::JointControlType::POSITION) {
        if (command.stiffness != 0.0 || command.damping != 0.0) {
          // 手动PD控制（用于motor执行器：腿、手臂等）
          ModelJointState& state = joint_states[i];
          position = command.stiffness * (command.position - state.position) - command.damping * state.velocity;
        } else {
          // 直接位置控制（用于position执行器：Hand等）
          position = command.position;
        }
      }

      if (command.actuator_id >= 0 && command.actuator_id < model_->nu) {
        data_->ctrl[command.actuator_id] = position;
      }
    }
  }
}

}  // namespace sim::mj
