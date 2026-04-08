#pragma once

#include <string>
#include <vector>

#include "./named.hpp"

namespace sim::data {

/**
 * @brief Represents the state of a joint.
 * @details The state of a joint includes the position, velocity, and effort.
 */
struct JointState {
  double position{0.0}; /**< The position of the joint. Unit rad or m. */
  double velocity{0.0}; /**< The velocity of the joint. Unit rad/s or m/s. */
  double effort{0.0};   /**< The effort applied to the joint. Unit NM or N. */
};

/**
 * @brief Represents the command for a joint.
 * @details The command for a joint includes the desired position, velocity, effort, stiffness, and damping.
 * @details MIT style, tau=Kp*(q_des-q)+Kv*(q_dot_des-q_dot)+tau_ff
 */
struct JointCommand {
  double position{0.0};  /**< The desired position for the joint. Unit rad or m. */
  double velocity{0.0};  /**< The desired velocity for the joint. Unit rad/s or m/s. */
  double effort{0.0};    /**< The desired effort for the joint. Unit N·m or N. */
  double stiffness{0.0}; /**< The stiffness of the joint. Unit N·m/rad or N/m */
  double damping{0.0};   /**< The damping of the joint. Unit N·s/m or N·m·s/rad*/
};

class JointParam {
 public:
  JointParam();
  ~JointParam() = default;

  std::string name;            /**< The name of the joint. */
  double position_upper_limit; /**< The upper limit of the joint position.  Unit rad or m. */
  double position_lower_limit; /**< The lower limit of the joint position. Unit rad or m. */
  double velocity_limit;       /**< The limit of the joint velocity. Unit rad/s or m/s. */
  double acceleration_limit;   /**< The limit of the joint acceleration. Unit rad/s^2 or m/s^2. */
  double effort_limit;         /**< The limit of the joint effort. Unit N·m or N. */
};

struct JointStates {
  std::vector<Named<JointState>> joints;
};

struct JointCommands {
  std::vector<Named<JointCommand>> joints;
};

}  // namespace sim::data
