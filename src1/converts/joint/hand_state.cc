// hand state converts impl
#include "./hand_state.h"
#include <chrono>

namespace sim::converts {

// SetHeader 辅助函数
inline void SetHeader(aimdk_msgs::msg::MessageHeader& header, const std::string& frame_id = "") {
  auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  header.frame_id = frame_id;
  header.stamp.sec = timestamp / 1e9;
  header.stamp.nanosec = timestamp % static_cast<uint64_t>(1e9);
}

// HandStateArray 转换函数：C++ JointStates -> ROS2 HandStateArray
// HandStateArray 采用对象数组(left_hands[]/right_hands[])结构
void Converts(aimdk_msgs::msg::HandStateArray& out, const sim::data::JointStates& in) {
  SetHeader(out.header);
  out.left_hands.clear();
  out.right_hands.clear();

  // 根据关节名称前缀分配到左右手
  for (const auto& joint : in.joints) {
    aimdk_msgs::msg::HandState state;
    // 关键修复：将 mujoco 模型的 "L_" / "R_" 前缀转换为 MC 的 "left_" / "right_" 前缀
    state.name = joint.name;
    if (state.name.find("L_") == 0) {
      state.name = "left_" + state.name.substr(2);  // 替换 "L_" 为 "left_"
    } else if (state.name.find("R_") == 0) {
      state.name = "right_" + state.name.substr(2);  // 替换 "R_" 为 "right_"
    }

    state.position = joint.position;
    state.velocity = joint.velocity;
    state.effort = joint.effort;
    // HandState 还有 state 和 faultcode 字段，这里设置为默认值
    state.state = 0;
    state.faultcode = 0;

    // 根据关节名称判断左右手 (left_ 前缀为左手, right_ 前缀为右手)
    if (state.name.find("left_") == 0) {
      out.left_hands.push_back(state);
    } else if (state.name.find("right_") == 0) {
      out.right_hands.push_back(state);
    }
  }

  // 设置手部类型
  // 重要：MC 的 HandCommandPublisher 要求 left_hand_type 和 right_hand_type 都不为 NONE 才会发布消息
  // 因此即使没有实际关节数据，也需要设置一个非 NONE 的类型，否则 MC 永远不会发布手部命令
  // 这里统一设置为 NIMBLE_HANDS，让 MC 能够正常发布命令
  out.left_hand_type.value = aimdk_msgs::msg::HandType::NIMBLE_HANDS;
  out.right_hand_type.value = aimdk_msgs::msg::HandType::NIMBLE_HANDS;
}

// HandStateArray 转换函数：ROS2 HandStateArray -> C++ JointStates
void Converts(sim::data::JointStates& out, const aimdk_msgs::msg::HandStateArray& in) {
  out.joints.clear();
  out.joints.reserve(in.left_hands.size() + in.right_hands.size());

  // 转换左手关节
  for (const auto& hand_state : in.left_hands) {
    sim::data::Named<sim::data::JointState> st;
    st.name = hand_state.name;
    st.position = hand_state.position;
    st.velocity = hand_state.velocity;
    st.effort = hand_state.effort;
    out.joints.push_back(std::move(st));
  }

  // 转换右手关节
  for (const auto& hand_state : in.right_hands) {
    sim::data::Named<sim::data::JointState> st;
    st.name = hand_state.name;
    st.position = hand_state.position;
    st.velocity = hand_state.velocity;
    st.effort = hand_state.effort;
    out.joints.push_back(std::move(st));
  }
}

}  // namespace sim::converts
