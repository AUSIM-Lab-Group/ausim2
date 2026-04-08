#pragma once

#include <mujoco/mujoco.h>
#include <simulate.h>

#include <memory>
#include <mutex>
#include <set>
#include <string>
#include "./model_info.h"

namespace sim::mj {

class ModelManager {
 public:
  ModelManager(std::shared_ptr<mujoco::Simulate> sim, const std::string& model_info);
  ~ModelManager();

  bool LoadModel(const std::string& simulation_model);

  mjModel* GetModel() const { return model_; }
  mjData* GetData() const { return data_; }

  void UpdateModelState();
  void UpdateModelCommand();

 private:
  void InitModel();
  void Reset();

 private:
  std::mutex data_mutex_;
  std::string model_info_;
  mjData* data_{nullptr};
  mjModel* model_{nullptr};
  std::shared_ptr<mujoco::Simulate> sim_{nullptr};
  std::set<sim::mj::JointFrameType> joint_frame_types_;
  std::set<sim::mj::SensorFrameType> sensor_frame_types_;
  bool has_raycaster_{false};
};

}  // namespace sim::mj
