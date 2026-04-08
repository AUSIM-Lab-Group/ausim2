#include <chrono>

#include "./joint_command.h"

namespace sim::converts {

#define CONVERTS_JOINT_HAS_FIELD(name)                                                     \
  template <typename T>                                                                    \
  struct joint_has_##name {                                                                \
   private:                                                                                \
    template <typename C>                                                                  \
    static constexpr auto check(C*) -> decltype(std::declval<C>().name, std::true_type{}); \
    template <typename>                                                                    \
    static constexpr std::false_type check(...);                                           \
                                                                                           \
   public:                                                                                 \
    static constexpr bool value = decltype(check<T>(nullptr))::value;                      \
  };

CONVERTS_JOINT_HAS_FIELD(stiffness)
CONVERTS_JOINT_HAS_FIELD(damping)

// inline void SetHeader(std_msgs::msg::MessageHeader& header, const std::string& frame_id = "") {
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
inline void ConvertsT1(T& out, const sim::data::JointCommands& in) {
  SetHeader(out.header);
  out.name.reserve(in.joints.size());
  out.position.reserve(in.joints.size());
  out.velocity.reserve(in.joints.size());
  out.effort.reserve(in.joints.size());

  if constexpr (joint_has_stiffness<T>::value) {
    out.stiffness.reserve(in.joints.size());
  }
  if constexpr (joint_has_damping<T>::value) {
    out.damping.reserve(in.joints.size());
  }

  for (int i = 0; i < in.joints.size(); ++i) {
    out.name.push_back(in.joints[i].name);
    out.position.push_back(in.joints[i].position);
    out.velocity.push_back(in.joints[i].velocity);
    out.effort.push_back(in.joints[i].effort);

    if constexpr (joint_has_stiffness<T>::value) {
      out.stiffness.push_back(in.joints[i].stiffness);
    }
    if constexpr (joint_has_damping<T>::value) {
      out.damping.push_back(in.joints[i].damping);
    }
  }
}

template <typename T>
inline void ConvertsT2(T& out, const sim::data::JointCommands& in) {
  // SetHeader(out.header);
  // 编译期获取 out.joints 包裹的类型
  using CMD = typename decltype(out.joints)::value_type;
  for (auto i = 0; i < in.joints.size(); ++i) {
    CMD command;
    command.name = in.joints[i].name;
    command.position = in.joints[i].position;
    command.velocity = in.joints[i].velocity;
    command.effort = in.joints[i].effort;

    // if constexpr (joint_has_stiffness<CMD>::value) {
    command.stiffness = in.joints[i].stiffness;
    // }
    // if constexpr (joint_has_damping<CMD>::value) {
    command.damping = in.joints[i].damping;
    // }

    // printf("name: %s, position: %f, velocity: %f, effort: %f, stiffness: %f, damping: %f\n", command.name.c_str(), command.position,
    // command.velocity,
    //        command.effort, command.stiffness, command.damping);

    out.joints.push_back(command);
  }
}

// void Converts(sensor_msgs::msg::JointState& out, const sim::data::JointCommands& in) { ConvertsT1(out, in); }
// void Converts(joint_state::msg::JointState& out, const sim::data::JointCommands& in) { ConvertsT1(out, in); }
// void Converts(joint_command::msg::JointCommand& out, const sim::data::JointCommands& in) { ConvertsT1(out, in); }
// void Converts(joint::msg::JointCommand& out, const sim::data::JointCommands& in) { ConvertsT2(out, in); }

template <typename T>
inline void ConvertsT1(sim::data::JointCommands& out, const T& in) {
  out.joints.resize(in.name.size());
  for (int i = 0; i < in.name.size(); ++i) {
    out.joints[i].name = in.name[i];
    out.joints[i].position = in.position[i];
    out.joints[i].velocity = in.velocity[i];
    out.joints[i].effort = in.effort[i];
    // if constexpr (joint_has_stiffness<T>::value) {
    out.joints[i].stiffness = in.stiffness[i];
    // }
    // if constexpr (joint_has_damping<T>::value) {
    out.joints[i].damping = in.damping[i];
    // }
  }
}

template <typename T>
inline void ConvertsT2(sim::data::JointCommands& out, const T& in) {
  out.joints.resize(in.joints.size());
  for (auto i = 0; i < in.joints.size(); ++i) {
    out.joints[i].name = in.joints[i].name;
    out.joints[i].position = in.joints[i].position;
    out.joints[i].velocity = in.joints[i].velocity;
    out.joints[i].effort = in.joints[i].effort;
    // if constexpr (joint_has_stiffness<T>::value) {
    out.joints[i].stiffness = in.joints[i].stiffness;
    // }
    // if constexpr (joint_has_damping<T>::value) {
    out.joints[i].damping = in.joints[i].damping;
    // }
  }
}

// void Converts(sim::data::JointCommands& out, const sensor_msgs::msg::JointState& in) { ConvertsT1(out, in); }
// void Converts(sim::data::JointCommands& out, const joint_state::msg::JointState& in) { ConvertsT1(out, in); }
// void Converts(sim::data::JointCommands& out, const joint_command::msg::JointCommand& in) { ConvertsT1(out, in); }
// void Converts(sim::data::JointCommands& out, const joint::msg::JointCommand& in) { ConvertsT2(out, in); }

// void Converts(agibot_z2_app::grasp::InspireState& out, const sim::api::data::FingerJointCommand& in) {
//   if (in.left.size() == 6) {
//     out.mutable_left_state()->set_pos_0(500);
//     out.mutable_left_state()->set_pos_1(in.left[1] * 1000);
//     out.mutable_left_state()->set_pos_2(in.left[2] * 1000);
//     out.mutable_left_state()->set_pos_3(in.left[3] * 1000);
//     out.mutable_left_state()->set_pos_4(in.left[4] * 1000);
//     out.mutable_left_state()->set_pos_5(in.left[5] * 1000);
//   }

//   if (in.right.size() == 6) {
//     out.mutable_right_state()->set_pos_0(500);
//     out.mutable_right_state()->set_pos_1(in.right[1] * 1000);
//     out.mutable_right_state()->set_pos_2(in.right[2] * 1000);
//     out.mutable_right_state()->set_pos_3(in.right[3] * 1000);
//     out.mutable_right_state()->set_pos_4(in.right[4] * 1000);
//     out.mutable_right_state()->set_pos_5(in.right[5] * 1000);
//   }
// }

void Converts(aimdk_msgs::msg::JointCommandArray& out, const data::JointCommands& in) { ConvertsT2(out, in); }

void Converts(data::JointCommands& out, const aimdk_msgs::msg::JointCommandArray& in) { ConvertsT2(out, in); }

}  // namespace sim::converts
