#pragma once

#include "config/quadrotor_config.hpp"
#include "manager/ros_bridge_process_manager.hpp"

namespace quadrotor {

class QuadrotorApp {
 public:
  QuadrotorApp(QuadrotorConfig config, RosBridgeLaunchConfig bridge_launch_config);

  int Run();

 private:
  QuadrotorConfig config_;
  RosBridgeLaunchConfig bridge_launch_config_;
};

}  // namespace quadrotor
