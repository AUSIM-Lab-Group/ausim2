#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "gui_runtime_config.hpp"

namespace {

void Expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}

void ExpectNear(double actual, double expected, const std::string& message) {
  constexpr double kTolerance = 1e-9;
  if (std::abs(actual - expected) > kTolerance) {
    std::cerr << message << ": expected " << expected << ", got " << actual << '\n';
    std::exit(1);
  }
}

std::filesystem::path TempPath(const std::string& name) {
  return std::filesystem::temp_directory_path() / name;
}

void WriteFile(const std::filesystem::path& path, const std::string& content) {
  std::ofstream out(path);
  Expect(out.good(), "failed to open temp config for writing");
  out << content;
}

std::string ReadFile(const std::filesystem::path& path) {
  std::ifstream in(path);
  Expect(in.good(), "failed to open temp config for reading");
  std::ostringstream content;
  content << in.rdbuf();
  return content.str();
}

std::string BaseConfig() {
  return R"yaml(
remote_control_node:
  ros__parameters:
    scale:
      linear:
        x: -0.6
        y: 0.7
        z: 0.8
      angular:
        yaw: -0.9
    keyboard:
      scale:
        linear:
          x: 0.3
          y: -0.4
          z: 0.5
        angular:
          yaw: 0.6
    gui:
      language: en
    actions:
      action1:
        service: /joy/action1
        buttons: [4, 0]
        keyboard: "t"
      action2:
        service: /joy/action2
        buttons: [5, 0]
        keyboard: "g"
      action3:
        service: /joy/action3
        buttons: [6, 7]
        keyboard: "x"
      action4:
        service: /joy/action4
        buttons: [-1]
        keyboard: "q"
      action5:
        service: ""
        buttons: [-1]
        keyboard: ""
      action6:
        service: ""
        buttons: [-1]
        keyboard: ""
      action7:
        service: /joy/action7
        buttons: [8]
        keyboard: "z"
)yaml";
}

void TestLoadEditableSettingsReadsRemoteControlNodeYaml() {
  const std::filesystem::path path = TempPath("remote_control_gui_runtime_load.yaml");
  WriteFile(path, BaseConfig());

  const remote_control::EditableGuiSettings settings = remote_control::LoadEditableGuiSettingsFromYaml(path.string());

  Expect(settings.language == "en", "language should be loaded from gui.language");
  ExpectNear(settings.joystick_scale.linear_x, -0.6, "joystick linear x scale should load");
  ExpectNear(settings.keyboard_scale.linear_y, -0.4, "keyboard linear y scale should load");
  Expect(settings.action_slots.size() == 6, "GUI should expose exactly six editable action slots");
  Expect(settings.action_slots[0].name == "action1", "first slot should be action1");
  Expect(settings.action_slots[0].service_name == "/joy/action1", "action1 service should load");
  Expect(settings.action_slots[0].buttons == std::vector<int>({4, 0}), "action1 buttons should load");
  Expect(settings.action_slots[3].buttons.empty(), "[-1] should load as an empty button binding");
  Expect(settings.action_slots[5].name == "action6", "sixth slot should be action6");
}

void TestApplyMaxVelocityPreservesDirectionSigns() {
  remote_control::EditableGuiSettings settings;
  settings.joystick_scale.linear_x = -0.6;
  settings.joystick_scale.linear_y = 0.7;
  settings.joystick_scale.linear_z = 0.8;
  settings.joystick_scale.angular_yaw = -0.9;
  settings.keyboard_scale.linear_x = 0.3;
  settings.keyboard_scale.linear_y = -0.4;
  settings.keyboard_scale.linear_z = 0.5;
  settings.keyboard_scale.angular_yaw = 0.6;

  remote_control::MotionScale limits;
  limits.linear_x = 1.2;
  limits.linear_y = 1.3;
  limits.linear_z = 1.4;
  limits.angular_yaw = 2.1;
  remote_control::ApplyMaxVelocityLimits(limits, &settings);

  ExpectNear(settings.joystick_scale.linear_x, -1.2, "negative joystick x direction should be preserved");
  ExpectNear(settings.joystick_scale.linear_y, 1.3, "positive joystick y direction should be preserved");
  ExpectNear(settings.joystick_scale.linear_z, 1.4, "positive joystick z direction should be preserved");
  ExpectNear(settings.joystick_scale.angular_yaw, -2.1, "negative joystick yaw direction should be preserved");
  ExpectNear(settings.keyboard_scale.linear_x, 1.2, "positive keyboard x direction should be preserved");
  ExpectNear(settings.keyboard_scale.linear_y, -1.3, "negative keyboard y direction should be preserved");
  ExpectNear(settings.keyboard_scale.linear_z, 1.4, "positive keyboard z direction should be preserved");
  ExpectNear(settings.keyboard_scale.angular_yaw, 2.1, "positive keyboard yaw direction should be preserved");
}

void TestSaveEditableSettingsWritesScalesLanguageAndSixActionSlots() {
  const std::filesystem::path path = TempPath("remote_control_gui_runtime_save.yaml");
  WriteFile(path, BaseConfig());

  remote_control::EditableGuiSettings settings = remote_control::LoadEditableGuiSettingsFromYaml(path.string());
  remote_control::MotionScale limits;
  limits.linear_x = 1.2;
  limits.linear_y = 1.3;
  limits.linear_z = 1.4;
  limits.angular_yaw = 2.1;
  remote_control::ApplyMaxVelocityLimits(limits, &settings);
  settings.language = "zh";
  settings.action_slots[4].service_name = "/joy/action5";
  settings.action_slots[4].buttons = {2, 3};
  settings.action_slots[4].keyboard_key = "h";
  settings.action_slots[5].service_name = "";
  settings.action_slots[5].buttons.clear();
  settings.action_slots[5].keyboard_key = "";

  remote_control::SaveEditableGuiSettingsToYaml(path.string(), settings);

  const YAML::Node root = YAML::LoadFile(path.string());
  const YAML::Node params = root["remote_control_node"]["ros__parameters"];
  ExpectNear(params["scale"]["linear"]["x"].as<double>(), -1.2, "saved joystick x scale should preserve sign");
  ExpectNear(params["keyboard"]["scale"]["linear"]["y"].as<double>(), -1.3, "saved keyboard y scale should preserve sign");
  ExpectNear(params["scale"]["angular"]["yaw"].as<double>(), -2.1, "saved joystick yaw scale should preserve sign");
  Expect(params["gui"]["language"].as<std::string>() == "zh", "saved language should be zh");
  Expect(params["actions"]["action5"]["service"].as<std::string>() == "/joy/action5", "action5 service should save");
  Expect(params["actions"]["action5"]["buttons"][0].as<int>() == 2, "action5 first button should save");
  Expect(params["actions"]["action5"]["buttons"][1].as<int>() == 3, "action5 second button should save");
  Expect(params["actions"]["action5"]["keyboard"].as<std::string>() == "h", "action5 keyboard should save");
  Expect(params["actions"]["action6"]["buttons"][0].as<int>() == -1, "empty action6 buttons should save as [-1]");
  Expect(params["actions"]["action7"]["service"].as<std::string>() == "/joy/action7", "action7 should be preserved but not edited");
}

void TestSaveEditableSettingsWritesWholeNumberScalesAsFloatLiterals() {
  const std::filesystem::path path = TempPath("remote_control_gui_runtime_float_literals.yaml");
  WriteFile(path, BaseConfig());

  remote_control::EditableGuiSettings settings = remote_control::LoadEditableGuiSettingsFromYaml(path.string());
  remote_control::MotionScale limits;
  limits.linear_x = 1.0;
  limits.linear_y = 1.0;
  limits.linear_z = 0.5;
  limits.angular_yaw = 2.0;
  remote_control::ApplyMaxVelocityLimits(limits, &settings);

  remote_control::SaveEditableGuiSettingsToYaml(path.string(), settings);

  const std::string saved = ReadFile(path);
  Expect(saved.find("x: -1.0") != std::string::npos, "whole-number signed joystick x scale should be emitted as float");
  Expect(saved.find("y: 1.0") != std::string::npos, "whole-number joystick y scale should be emitted as float");
  Expect(saved.find("yaw: -2.0") != std::string::npos, "whole-number joystick yaw scale should be emitted as float");
}

void TestLoadEditableSettingsToleratesMissingOptionalBlocks() {
  const std::filesystem::path path = TempPath("remote_control_gui_runtime_sparse.yaml");
  WriteFile(path, R"yaml(
remote_control_node:
  ros__parameters:
    topics:
      joy: /joy
)yaml");

  const remote_control::EditableGuiSettings settings = remote_control::LoadEditableGuiSettingsFromYaml(path.string());

  Expect(settings.language == "zh", "missing gui.language should default to zh");
  ExpectNear(settings.joystick_scale.linear_x, 0.6, "missing scale should keep default joystick x");
  ExpectNear(settings.keyboard_scale.angular_yaw, 1.0, "missing keyboard scale should keep default keyboard yaw");
  Expect(settings.action_slots.size() == 6, "missing actions should keep six default action slots");
}

}  // namespace

int main() {
  TestLoadEditableSettingsReadsRemoteControlNodeYaml();
  TestApplyMaxVelocityPreservesDirectionSigns();
  TestSaveEditableSettingsWritesScalesLanguageAndSixActionSlots();
  TestSaveEditableSettingsWritesWholeNumberScalesAsFloatLiterals();
  TestLoadEditableSettingsToleratesMissingOptionalBlocks();
  return 0;
}
