#include <chrono>

#include "./hand_command.h"

namespace sim::converts {
inline void SetHeader(aimdk_msgs::msg::MessageHeader& header, const std::string& frame_id = "") {
  auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  header.frame_id = frame_id;
  header.stamp.sec = timestamp / 1e9;
  header.stamp.nanosec = timestamp % static_cast<uint64_t>(1e9);
}

// HandCommandArray 转换函数：C++ JointCommands -> ROS2 HandCommandArray
void ConvertsToHandCommandArray(aimdk_msgs::msg::HandCommandArray& out, const data::JointCommands& in) {
  SetHeader(out.header);
  out.left_hands.clear();
  out.right_hands.clear();

  // 根据关节名称前缀分配到左右手
  for (const auto& joint : in.joints) {
    aimdk_msgs::msg::HandCommand cmd;
    cmd.name = joint.name;
    cmd.position = joint.position;
    cmd.velocity = joint.velocity;
    cmd.effort = joint.effort;
    cmd.acceleration = 1.0;  // 默认值或从其他字段映射
    cmd.deceleration = 1.0;  // 默认值或从其他字段映射

    // 根据关节名称判断左右手 (L_ 前缀为左手, R_ 前缀为右手)
    if (joint.name.size() > 0 && joint.name[0] == 'L') {
      out.left_hands.push_back(cmd);
    } else if (joint.name.size() > 0 && joint.name[0] == 'R') {
      out.right_hands.push_back(cmd);
    }
  }

  // 设置手部类型
  // 重要：MC 的 HandCommandPublisher 要求 left_hand_type 和 right_hand_type 都不为 NONE 才会发布消息
  // 因此即使没有实际关节数据，也需要设置一个非 NONE 的类型，否则 MC 永远不会发布手部命令
  // 这里统一设置为 NIMBLE_HANDS，让 MC 能够正常发布命令
  out.left_hand_type.value = aimdk_msgs::msg::HandType::NIMBLE_HANDS;
  out.right_hand_type.value = aimdk_msgs::msg::HandType::NIMBLE_HANDS;
}

// Backwards-compatible wrappers: keep the old `Converts` API and forward to clearer names.
void Converts(aimdk_msgs::msg::HandCommandArray& out, const data::JointCommands& in) { ConvertsToHandCommandArray(out, in); }

// HandCommandArray 转换函数：ROS2 HandCommandArray -> C++ JointCommands
void ConvertsFromHandCommandArray(data::JointCommands& out, const aimdk_msgs::msg::HandCommandArray& in) {
  out.joints.clear();
  out.joints.reserve(in.left_hands.size() + in.right_hands.size());

  // 转换左手关节
  for (const auto& hand_cmd : in.left_hands) {
    sim::data::Named<sim::data::JointCommand> cmd;
    // 关键修复：将 MC 的 "left_" 前缀转换为 mujoco 模型的 "L_" 前缀
    cmd.name = hand_cmd.name;
    if (cmd.name.find("left_") == 0) {
      cmd.name = "L_" + cmd.name.substr(5);  // 替换 "left_" 为 "L_"
    }
    cmd.position = hand_cmd.position;
    cmd.velocity = hand_cmd.velocity;
    cmd.effort = hand_cmd.effort;
    cmd.stiffness = 0.0;
    cmd.damping = 0.0;
    out.joints.push_back(cmd);
  }

  // 转换右手关节
  for (const auto& hand_cmd : in.right_hands) {
    sim::data::Named<sim::data::JointCommand> cmd;
    // 关键修复：将 MC 的 "right_" 前缀转换为 mujoco 模型的 "R_" 前缀
    cmd.name = hand_cmd.name;
    if (cmd.name.find("right_") == 0) {
      cmd.name = "R_" + cmd.name.substr(6);  // 替换 "right_" 为 "R_"
    }
    cmd.position = hand_cmd.position;
    cmd.velocity = hand_cmd.velocity;
    cmd.effort = hand_cmd.effort;
    cmd.stiffness = 0.0;
    cmd.damping = 0.0;
    out.joints.push_back(cmd);
  }
}

// Backwards-compatible wrapper
void Converts(data::JointCommands& out, const aimdk_msgs::msg::HandCommandArray& in) { ConvertsFromHandCommandArray(out, in); }

}  // namespace sim::converts