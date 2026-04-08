#pragma once

#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include "aima/sim/data/imu.hpp"

namespace sim::converts {

void Converts(sim::data::ImuData& out, const sensor_msgs::msg::Imu& in);
void Converts(sensor_msgs::msg::Imu& out, const sim::data::ImuData& in);

void Converts(std_msgs::msg::Float64MultiArray& out, const std::vector<std::vector<double>>& in);

// imu data -> tf message
void Converts(tf2_msgs::msg::TFMessage& out, const sim::data::ImuData& in);

}  // namespace sim::converts
