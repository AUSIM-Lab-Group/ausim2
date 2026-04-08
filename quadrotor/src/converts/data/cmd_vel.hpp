#pragma once

#include <chrono>

#include "data/cmd_vel.hpp"
#include "runtime/runtime_types.hpp"

#if defined(QUADROTOR_HAS_ROS2)
#include <geometry_msgs/msg/twist.hpp>
#endif

namespace quadrotor::converts {

VelocityCommand ToVelocityCommand(
    const data::CmdVelData& in,
    std::chrono::steady_clock::time_point received_time);

#if defined(QUADROTOR_HAS_ROS2)
void Convert(data::CmdVelData& out, const geometry_msgs::msg::Twist& in);
data::CmdVelData ToCmdVelData(const geometry_msgs::msg::Twist& in);
void Convert(geometry_msgs::msg::Twist& out, const data::CmdVelData& in);
geometry_msgs::msg::Twist ToRosMessage(const data::CmdVelData& in);
#endif

}  // namespace quadrotor::converts
