#pragma once

#include "aima/sim/data/joint.hpp"
// #include "aimdk/protocol/common/joint.h"
// #include "aimdk/protocol/common/joint.pb.h"
// #include "aimdk/protocol/hal/joint/joint_channel.pb.h"

#include <aimdk_msgs/msg/joint_state.hpp>
#include <aimdk_msgs/msg/joint_state_array.hpp>

// #include <sensor_msgs/msg/joint_state.hpp>
// #include "hal_msgs/msg/joint_state.hpp"
// #include "joint_state/msg/joint_state.hpp"

namespace sim::converts {

// void Converts(sensor_msgs::msg::JointState& out, const sim::data::JointStates& in);
// void Converts(joint_state::msg::JointState& out, const sim::data::JointStates& in);
// void Converts(::joint::msg::JointState& out, const sim::data::JointStates& in);
// void Converts(aimdk::protocol::JointState& out, const sim::data::Named<sim::data::JointState>& in);

// void Converts(sim::data::JointStates& out, const sensor_msgs::msg::JointState& in);
// void Converts(sim::data::JointStates& out, const joint_state::msg::JointState& in);
// void Converts(sim::data::JointStates& out, const ::joint::msg::JointState& in);
// void Converts(sim::data::JointStates& out, const aimdk::protocol::JointStatesChannel& in);
// void Converts(sim::data::Named<sim::data::JointState>& out, const aimdk::protocol::JointState& in);

void Converts(aimdk_msgs::msg::JointStateArray& out, const sim::data::JointStates& in);
void Converts(sim::data::JointStates& out, const aimdk_msgs::msg::JointStateArray& in);
}  // namespace sim::converts
