#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "gui_control_model.hpp"

namespace remote_control {

constexpr std::size_t kEditableActionSlotCount = 6;

struct GuiActionSlotConfig {
  std::string name;
  std::string service_name;
  std::vector<int> buttons;
  std::string keyboard_key;
};

struct JoystickAxisMapping {
  int linear_x = 4;
  int linear_y = 0;
  int linear_z = 1;
  int angular_yaw = 3;
};

struct GuiWindowSettings {
  int width = 920;
  int height = 760;
};

struct EditableGuiSettings {
  GuiWindowSettings window;
  JoystickAxisMapping axis_mapping;
  MotionScale joystick_scale;
  MotionScale keyboard_scale;
  std::string language = "zh";
  std::vector<GuiActionSlotConfig> action_slots;
};

std::vector<GuiActionSlotConfig> DefaultEditableActionSlots();
EditableGuiSettings LoadEditableGuiSettingsFromYaml(const std::string& config_path);
void SaveEditableGuiSettingsToYaml(const std::string& config_path, const EditableGuiSettings& settings);
MotionScale CurrentMaxVelocityLimits(const EditableGuiSettings& settings);
void ApplyMaxVelocityLimits(const MotionScale& limits, EditableGuiSettings* settings);

}  // namespace remote_control
