#pragma once

#include <string>

#include "data/transform.hpp"
#include "runtime/runtime_types.hpp"

#if defined(QUADROTOR_HAS_ROS2)
#include <geometry_msgs/msg/transform_stamped.hpp>
#endif

namespace quadrotor::converts {

data::TransformData ToTransformData(
    const TelemetrySnapshot& snapshot,
    const std::string& frame_id,
    const std::string& child_frame_id);

#if defined(QUADROTOR_HAS_ROS2)
void Convert(geometry_msgs::msg::TransformStamped& out, const data::TransformData& in);
geometry_msgs::msg::TransformStamped ToRosMessage(const data::TransformData& in);
#endif

}  // namespace quadrotor::converts
