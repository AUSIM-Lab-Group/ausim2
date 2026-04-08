#pragma once

namespace sim::data {

/**
 * @brief Represents the state of an IMU.
 * @details The state of an IMU includes the acceleration, angular velocity, and magnetic field.
 */

struct Vec3 {
  double x;
  double y;
  double z;
};

struct Quaternion {
  double w;
  double x;
  double y;
  double z;
};

struct ImuData {
  Vec3 angular_velocity;
  Vec3 linear_acceleration;
  Vec3 linear_position;
  Quaternion orientation;  ///< The quaternion of the pose.
};

}  // namespace sim::data