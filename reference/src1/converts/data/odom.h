// sim_mujoco_copy/converts/data/odom.h
#pragma once

#include <nav_msgs/msg/odometry.hpp>
#include "aima/sim/data/odom.hpp"

namespace sim::converts {
// 将sim的OdomData转换为ROS的Odometry消息
void Converts(nav_msgs::msg::Odometry& out, const sim::data::OdomData& in);
}  // namespace sim::converts