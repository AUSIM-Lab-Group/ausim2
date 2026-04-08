// hand state converts header
#pragma once

#include <aimdk_msgs/msg/hand_state_array.hpp>
#include "aima/sim/data/joint.hpp"

namespace sim::converts {

// HandStateArray 转换函数：C++ JointStates <-> ROS2 HandStateArray
// 注意：HandStateArray 使用对象数组结构 (left_hands[]/right_hands[])
void Converts(aimdk_msgs::msg::HandStateArray& out, const sim::data::JointStates& in);
void Converts(sim::data::JointStates& out, const aimdk_msgs::msg::HandStateArray& in);

}  // namespace sim::converts
