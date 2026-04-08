#pragma once

#include <array>
#include <string>

#include <mujoco/mujoco.h>

#include "config/quadrotor_config.hpp"

namespace quadrotor {

struct SensorBinding {
  std::string name;
  int id = -1;
  int adr = -1;
  int dim = 0;
  bool resolved = false;
};

class MujocoBindings {
 public:
  explicit MujocoBindings(const QuadrotorConfig& config);

  std::string ValidateModel(const mjModel* model) const;
  void Resolve(const mjModel* model);

  const std::array<int, 4>& motor_actuator_ids() const { return motor_actuator_ids_; }
  int vehicle_body_id() const { return vehicle_body_id_; }
  int track_camera_id() const { return track_camera_id_; }
  const SensorBinding& gyro_sensor() const { return gyro_sensor_; }
  const SensorBinding& accelerometer_sensor() const { return accelerometer_sensor_; }
  const SensorBinding& quaternion_sensor() const { return quaternion_sensor_; }

 private:
  void ResolveSensor(const mjModel* model, SensorBinding* binding);

  std::array<std::string, 4> motor_names_;
  std::array<int, 4> motor_actuator_ids_ = {-1, -1, -1, -1};
  std::string vehicle_body_name_;
  std::string track_camera_name_;
  SensorBinding gyro_sensor_;
  SensorBinding accelerometer_sensor_;
  SensorBinding quaternion_sensor_;
  int vehicle_body_id_ = -1;
  int track_camera_id_ = -1;
};

}  // namespace quadrotor
