#pragma once

#include <aimdk_msgs/msg/hand_command_array.hpp>
#include "aima/sim/data/joint.hpp"

namespace sim::converts {

// HandCommandArray 转换函数（更明确的命名）：C++ JointCommands <-> ROS2 HandCommandArray
// 注意：HandCommandArray 使用对象数组结构 (left_hands[]/right_hands[])
// - ConvertsToHandCommandArray: 将内部 JointCommands 转换为 ROS2 HandCommandArray
// - ConvertsFromHandCommandArray: 将 ROS2 HandCommandArray 转换为内部 JointCommands
// New clearer names (preferred):
void ConvertsToHandCommandArray(aimdk_msgs::msg::HandCommandArray& out, const data::JointCommands& in);
void ConvertsFromHandCommandArray(data::JointCommands& out, const aimdk_msgs::msg::HandCommandArray& in);

// Backwards-compatible overloads (existing code calls `Converts(...)`).
// These forward to the clearer functions above. Keeping them avoids breaking callers.
void Converts(aimdk_msgs::msg::HandCommandArray& out, const data::JointCommands& in);
void Converts(data::JointCommands& out, const aimdk_msgs::msg::HandCommandArray& in);

}  // namespace sim::converts