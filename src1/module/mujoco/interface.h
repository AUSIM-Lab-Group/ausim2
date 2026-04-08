#pragma once

#include "aima/sim/data/imu.hpp"
#include "aima/sim/data/joint.hpp"
#include "aima/sim/data/odom.hpp"
#include "aima/sim/data/raycaster.hpp"

namespace sim::mj {

void GetJointState(sim::data::JointStates& states);
void GetArmJointState(sim::data::JointStates& states);
void GetHeadJointState(sim::data::JointStates& states);
void GetLegJointState(sim::data::JointStates& states);
void GetLiftJointState(sim::data::JointStates& states);
void GetWaistJointState(sim::data::JointStates& states);
void GetWheelJointState(sim::data::JointStates& states);
void GetHandJointState(sim::data::JointStates& states);

void SetJointCommand(sim::data::JointCommands& commands);
void SetArmJointCommand(sim::data::JointCommands& commands);
void SetLegJointCommand(sim::data::JointCommands& commands);
void SetLiftJointCommand(sim::data::JointCommands& commands);
void SetWaistJointCommand(sim::data::JointCommands& commands);
void SetWheelJointCommand(sim::data::JointCommands& commands);
void SetHeadJointCommand(sim::data::JointCommands& commands);
void SetHandJointCommand(sim::data::JointCommands& commands);
void GetImuData(sim::data::ImuData& imu_data, std::string imu_frame_type);
void GetRaycasterData(sim::data::RaycasterData& raycaster_data, std::string raycaster_name);
void GetOdomData(sim::data::OdomData& odom_data);
// void GetImuData(sim::data::ImuData& imu_data);
// void GetFootSensorData(std::vector<std::vector<double>>& foot_sensor_data);

}  // namespace sim::mj
