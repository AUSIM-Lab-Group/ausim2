#pragma once

// #include <sensor_msgs/msg/joint_state.hpp>

// #include "joint_command/msg/joint_command.hpp"
// #include "joint/msg/joint_state.hpp"
// #include "joint_state/msg/joint_state.hpp"

#include "aima/sim/data/joint.hpp"

#include <aimdk_msgs/msg/joint_command.hpp>
#include <aimdk_msgs/msg/joint_command_array.hpp>

namespace sim::converts {
void Converts(aimdk_msgs::msg::JointCommandArray& out, const data::JointCommands& in);
void Converts(data::JointCommands& out, const aimdk_msgs::msg::JointCommandArray& in);

}  // namespace sim::converts
