#include "./joint_state.h"

#include <chrono>

namespace sim::converts {

// inline void SetHeader(std_msgs::msg::Header& header, const std::string& frame_id = "") {
//   auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
//   header.frame_id = frame_id;
//   header.stamp.sec = timestamp / 1e9;
//   header.stamp.nanosec = timestamp % static_cast<uint64_t>(1e9);
// }

inline void SetHeader(aimdk_msgs::msg::MessageHeader& header, const std::string& frame_id = "") {
  auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  header.frame_id = frame_id;
  header.stamp.sec = timestamp / 1e9;
  header.stamp.nanosec = timestamp % static_cast<uint64_t>(1e9);
}

template <typename T>
inline void Converts1(T& out, const sim::data::JointStates& in) {
  SetHeader(out.header);
  out.name.reserve(in.joints.size());
  out.position.reserve(in.joints.size());
  out.velocity.reserve(in.joints.size());
  out.effort.reserve(in.joints.size());

  for (const auto& state : in.joints) {
    out.name.push_back(state.name);
    out.position.push_back(state.position);
    out.velocity.push_back(state.velocity);
    out.effort.push_back(state.effort);
  }
}

template <typename T>
inline void Converts2(T& out, const sim::data::JointStates& in) {
  SetHeader(out.header);
  // 编译期获取 out.joints 包裹的类型
  using STATE = typename decltype(out.joints)::value_type;
  for (auto i = 0; i < in.joints.size(); ++i) {
    STATE state;
    state.name = in.joints[i].name;
    state.position = in.joints[i].position;
    state.velocity = in.joints[i].velocity;
    state.effort = in.joints[i].effort;
    out.joints.push_back(state);
  }
}

// void Converts(sensor_msgs::msg::JointState& out, const sim::data::JointStates& in) { Converts1(out, in); }
// void Converts(joint_state::msg::JointState& out, const sim::data::JointStates& in) { Converts1(out, in); }
// void Converts(joint::msg::JointState& out, const sim::data::JointStates& in) { Converts2(out, in); }

template <typename T>
inline void Converts1(sim::data::JointStates& out, const T& in) {
  out.joints.reserve(in.name.size());
  using STATE = typename decltype(out.joints)::value_type;
  for (auto i = 0; i < in.name.size(); ++i) {
    STATE state;
    state.name = in.name[i];
    state.position = in.position[i];
    state.velocity = in.velocity[i];
    state.effort = in.effort[i];
    out.joints.push_back(state);
  }
}

template <typename T>
inline void Converts2(sim::data::JointStates& out, const T& in) {
  out.joints.reserve(in.joints.size());
  using STATE = typename decltype(out.joints)::value_type;
  for (auto& joint : in.joints) {
    STATE state;
    state.name = joint.name;
    state.position = joint.position;
    state.velocity = joint.velocity;
    state.effort = joint.effort;
    out.joints.push_back(state);
  }
}

// void Converts(sim::data::JointStates& out, const sensor_msgs::msg::JointState& in) { Converts1(out, in); }
// void Converts(sim::data::JointStates& out, const joint_state::msg::JointState& in) { Converts1(out, in); }
// void Converts(sim::data::JointStates& out, const joint::msg::JointState& in) { Converts2(out, in); }

// void Converts(aimdk::protocol::JointState& out, const sim::data::Named<sim::data::JointState>& in) {
//   out.set_name(in.name);
//   out.set_position(in.position);
//   out.set_velocity(in.velocity);
//   out.set_effort(in.effort);
// }

// void Converts(sim::data::Named<sim::data::JointState>& out, const aimdk::protocol::JointState& in) {
//   out.name = in.name();
//   out.position = in.position();
//   out.velocity = in.velocity();
//   out.effort = in.effort();
// }

// void Converts(sim::data::JointStates& out, const aimdk::protocol::JointStatesChannel& in) {
//   out.joints.resize(in.states().size());
//   for (int i = 0; i < in.states().size(); ++i) {
//     Converts(out.joints[i], in.states(i));
//   }
// }
void Converts(aimdk_msgs::msg::JointStateArray& out, const sim::data::JointStates& in) { Converts2(out, in); }
void Converts(sim::data::JointStates& out, const aimdk_msgs::msg::JointStateArray& in) { Converts2(out, in); }
}  // namespace sim::converts
