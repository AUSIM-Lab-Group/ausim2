#include "gui_runtime_config.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace remote_control {
namespace {

YAML::Node LoadYamlFileOrEmpty(const std::string& config_path) {
  if (config_path.empty()) {
    return YAML::Node(YAML::NodeType::Map);
  }
  if (!std::filesystem::exists(config_path)) {
    return YAML::Node(YAML::NodeType::Map);
  }
  YAML::Node root = YAML::LoadFile(config_path);
  if (!root || !root.IsMap()) {
    return YAML::Node(YAML::NodeType::Map);
  }
  return root;
}

YAML::Node FindParameterRoot(const YAML::Node& root) {
  const char* node_names[] = {"remote_control_gui_node", "remote_control_node"};
  for (const char* node_name : node_names) {
    const YAML::Node node = root[node_name];
    if (!node || !node.IsMap()) {
      continue;
    }
    const YAML::Node params = node["ros__parameters"];
    if (params && params.IsMap()) {
      return params;
    }
  }
  const YAML::Node wildcard = root["/**"];
  if (wildcard && wildcard.IsMap()) {
    const YAML::Node wildcard_params = wildcard["ros__parameters"];
    if (wildcard_params && wildcard_params.IsMap()) {
      return wildcard_params;
    }
  }
  return YAML::Node();
}

YAML::Node MutableParameterRoot(YAML::Node* root) {
  const char* node_names[] = {"remote_control_gui_node", "remote_control_node"};
  const YAML::Node const_root = *root;
  for (const char* node_name : node_names) {
    const YAML::Node node = const_root[node_name];
    if (!node || !node.IsMap()) {
      continue;
    }
    const YAML::Node params = node["ros__parameters"];
    if (params && params.IsMap()) {
      return (*root)[node_name]["ros__parameters"];
    }
  }
  (*root)["remote_control_node"]["ros__parameters"] = YAML::Node(YAML::NodeType::Map);
  return (*root)["remote_control_node"]["ros__parameters"];
}

YAML::Node Child(const YAML::Node& node, const char* key) {
  if (!node || !node.IsMap()) {
    return YAML::Node();
  }
  return node[key];
}

double ReadDouble(const YAML::Node& node, double fallback) {
  if (!node || !node.IsScalar()) {
    return fallback;
  }
  try {
    return node.as<double>();
  } catch (const YAML::Exception&) {
    return fallback;
  }
}

int ReadInt(const YAML::Node& node, int fallback) {
  if (!node || !node.IsScalar()) {
    return fallback;
  }
  try {
    return node.as<int>();
  } catch (const YAML::Exception&) {
    return fallback;
  }
}

bool ReadBool(const YAML::Node& node, bool fallback) {
  if (!node || !node.IsScalar()) {
    return fallback;
  }
  try {
    return node.as<bool>();
  } catch (const YAML::Exception&) {
    return fallback;
  }
}

std::string ReadString(const YAML::Node& node, const std::string& fallback) {
  if (!node || !node.IsScalar()) {
    return fallback;
  }
  try {
    return node.as<std::string>();
  } catch (const YAML::Exception&) {
    return fallback;
  }
}

GuiSectionSettings ReadSectionSettings(const YAML::Node& node, GuiSectionSettings fallback) {
  fallback.joystick_state_expanded = ReadBool(Child(node, "joystick_state_expanded"), fallback.joystick_state_expanded);
  fallback.cmd_vel_mapping_expanded = ReadBool(Child(node, "cmd_vel_mapping_expanded"), fallback.cmd_vel_mapping_expanded);
  fallback.action_mapping_expanded = ReadBool(Child(node, "action_mapping_expanded"), fallback.action_mapping_expanded);
  fallback.action_history_expanded = ReadBool(Child(node, "action_history_expanded"), fallback.action_history_expanded);
  return fallback;
}

JoystickAxisMapping ReadAxisMapping(const YAML::Node& node, JoystickAxisMapping fallback) {
  const YAML::Node linear = Child(node, "linear");
  const YAML::Node angular = Child(node, "angular");
  fallback.linear_x = ReadInt(Child(linear, "x"), fallback.linear_x);
  fallback.linear_y = ReadInt(Child(linear, "y"), fallback.linear_y);
  fallback.linear_z = ReadInt(Child(linear, "z"), fallback.linear_z);
  fallback.angular_yaw = ReadInt(Child(angular, "yaw"), fallback.angular_yaw);
  return fallback;
}

GuiWindowSettings ReadWindowSettings(const YAML::Node& node, GuiWindowSettings fallback) {
  fallback.width = std::max(920, ReadInt(Child(node, "width"), fallback.width));
  fallback.height = std::max(640, ReadInt(Child(node, "height"), fallback.height));
  return fallback;
}

MotionScale ReadMotionScale(const YAML::Node& node, MotionScale fallback) {
  const YAML::Node linear = Child(node, "linear");
  const YAML::Node angular = Child(node, "angular");
  fallback.linear_x = ReadDouble(Child(linear, "x"), fallback.linear_x);
  fallback.linear_y = ReadDouble(Child(linear, "y"), fallback.linear_y);
  fallback.linear_z = ReadDouble(Child(linear, "z"), fallback.linear_z);
  fallback.angular_yaw = ReadDouble(Child(angular, "yaw"), fallback.angular_yaw);
  return fallback;
}

void WriteAxisMapping(YAML::Node node, const JoystickAxisMapping& mapping) {
  node["linear"]["x"] = mapping.linear_x;
  node["linear"]["y"] = mapping.linear_y;
  node["linear"]["z"] = mapping.linear_z;
  node["angular"]["yaw"] = mapping.angular_yaw;
}

void WriteWindowSettings(YAML::Node node, const GuiWindowSettings& window) {
  node["width"] = std::max(920, window.width);
  node["height"] = std::max(640, window.height);
}

void WriteSectionSettings(YAML::Node node, const GuiSectionSettings& sections) {
  node["joystick_state_expanded"] = sections.joystick_state_expanded;
  node["cmd_vel_mapping_expanded"] = sections.cmd_vel_mapping_expanded;
  node["action_mapping_expanded"] = sections.action_mapping_expanded;
  node["action_history_expanded"] = sections.action_history_expanded;
}

std::string FloatLiteral(double value) {
  std::ostringstream out;
  out << std::fixed << std::setprecision(6) << value;
  std::string text = out.str();
  while (text.size() > 3 && text.back() == '0' && text[text.size() - 2] != '.') {
    text.pop_back();
  }
  return text;
}

void WriteMotionScale(YAML::Node node, const MotionScale& scale) {
  node["linear"]["x"] = FloatLiteral(scale.linear_x);
  node["linear"]["y"] = FloatLiteral(scale.linear_y);
  node["linear"]["z"] = FloatLiteral(scale.linear_z);
  node["angular"]["yaw"] = FloatLiteral(scale.angular_yaw);
}

std::string NormalizeLanguage(std::string language) {
  std::transform(language.begin(), language.end(), language.begin(), [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
  return language == "en" ? "en" : "zh";
}

std::string NormalizeKeyboardKey(const std::string& key) {
  if (key.empty()) {
    return "";
  }
  const char first = key.front();
  if (first == ' ') {
    return " ";
  }
  return std::string(1, static_cast<char>(std::tolower(static_cast<unsigned char>(first))));
}

std::vector<int> ReadButtons(const YAML::Node& node) {
  std::vector<int> buttons;
  if (!node || !node.IsSequence()) {
    return buttons;
  }
  for (const YAML::Node& item : node) {
    try {
      const int button = item.as<int>();
      if (button >= 0) {
        buttons.push_back(button);
      }
    } catch (const YAML::Exception&) {
    }
  }
  return buttons;
}

YAML::Node ButtonsToYaml(const std::vector<int>& buttons) {
  YAML::Node node(YAML::NodeType::Sequence);
  if (buttons.empty()) {
    node.push_back(-1);
    return node;
  }
  for (const int button : buttons) {
    if (button >= 0) {
      node.push_back(button);
    }
  }
  if (node.size() == 0) {
    node.push_back(-1);
  }
  return node;
}

double PreserveSignedMagnitude(double current, double limit) {
  const double magnitude = std::abs(limit);
  return current < 0.0 ? -magnitude : magnitude;
}

void WriteYamlAtomically(const std::string& config_path, const YAML::Node& root) {
  if (config_path.empty()) {
    throw std::runtime_error("config path is empty");
  }

  const std::filesystem::path target(config_path);
  const std::filesystem::path temp(target.string() + ".tmp");
  {
    std::ofstream out(temp);
    if (!out.good()) {
      throw std::runtime_error("failed to open temporary config for writing: " + temp.string());
    }
    out << root << '\n';
    if (!out.good()) {
      throw std::runtime_error("failed to write temporary config: " + temp.string());
    }
  }
  std::filesystem::rename(temp, target);
}

}  // namespace

std::vector<GuiActionSlotConfig> DefaultEditableActionSlots() {
  return {
      {"action1", "/joy/action1", {4, 0}, "t"},
      {"action2", "/joy/action2", {5, 0}, "g"},
      {"action3", "/joy/action3", {6, 7}, "x"},
      {"action4", "/joy/action4", {}, "q"},
      {"action5", "", {}, ""},
      {"action6", "", {}, ""},
  };
}

EditableGuiSettings LoadEditableGuiSettingsFromYaml(const std::string& config_path) {
  EditableGuiSettings settings;
  settings.action_slots = DefaultEditableActionSlots();

  const YAML::Node root = LoadYamlFileOrEmpty(config_path);
  const YAML::Node params = FindParameterRoot(root);
  if (!params) {
    return settings;
  }

  settings.axis_mapping = ReadAxisMapping(Child(params, "axes"), settings.axis_mapping);
  settings.joystick_scale = ReadMotionScale(Child(params, "scale"), settings.joystick_scale);
  settings.keyboard_scale = ReadMotionScale(Child(Child(params, "keyboard"), "scale"), settings.keyboard_scale);
  settings.language = NormalizeLanguage(ReadString(Child(Child(params, "gui"), "language"), settings.language));
  settings.window = ReadWindowSettings(Child(Child(params, "gui"), "window"), settings.window);
  settings.sections = ReadSectionSettings(Child(Child(params, "gui"), "sections"), settings.sections);

  const YAML::Node actions = Child(params, "actions");
  if (actions && actions.IsMap()) {
    for (GuiActionSlotConfig& slot : settings.action_slots) {
      const YAML::Node action = Child(actions, slot.name.c_str());
      if (!action || !action.IsMap()) {
        continue;
      }
      slot.service_name = ReadString(Child(action, "service"), slot.service_name);
      slot.buttons = ReadButtons(Child(action, "buttons"));
      slot.keyboard_key = NormalizeKeyboardKey(ReadString(Child(action, "keyboard"), slot.keyboard_key));
    }
  }

  return settings;
}

void SaveEditableGuiSettingsToYaml(const std::string& config_path, const EditableGuiSettings& settings) {
  YAML::Node root = LoadYamlFileOrEmpty(config_path);
  YAML::Node params = MutableParameterRoot(&root);

  WriteAxisMapping(params["axes"], settings.axis_mapping);
  WriteMotionScale(params["scale"], settings.joystick_scale);
  WriteMotionScale(params["keyboard"]["scale"], settings.keyboard_scale);
  params["gui"]["language"] = NormalizeLanguage(settings.language);
  WriteWindowSettings(params["gui"]["window"], settings.window);
  WriteSectionSettings(params["gui"]["sections"], settings.sections);

  const std::vector<GuiActionSlotConfig> defaults = DefaultEditableActionSlots();
  const std::size_t count = std::min(defaults.size(), settings.action_slots.size());
  for (std::size_t index = 0; index < count; ++index) {
    const GuiActionSlotConfig& slot = settings.action_slots[index];
    const std::string name = slot.name.empty() ? defaults[index].name : slot.name;
    params["actions"][name]["service"] = slot.service_name;
    params["actions"][name]["buttons"] = ButtonsToYaml(slot.buttons);
    params["actions"][name]["keyboard"] = NormalizeKeyboardKey(slot.keyboard_key);
  }

  WriteYamlAtomically(config_path, root);
}

MotionScale CurrentMaxVelocityLimits(const EditableGuiSettings& settings) {
  MotionScale limits;
  limits.linear_x = std::abs(settings.joystick_scale.linear_x);
  limits.linear_y = std::abs(settings.joystick_scale.linear_y);
  limits.linear_z = std::abs(settings.joystick_scale.linear_z);
  limits.angular_yaw = std::abs(settings.joystick_scale.angular_yaw);
  return limits;
}

void ApplyMaxVelocityLimits(const MotionScale& limits, EditableGuiSettings* settings) {
  if (settings == nullptr) {
    return;
  }
  settings->joystick_scale.linear_x = PreserveSignedMagnitude(settings->joystick_scale.linear_x, limits.linear_x);
  settings->joystick_scale.linear_y = PreserveSignedMagnitude(settings->joystick_scale.linear_y, limits.linear_y);
  settings->joystick_scale.linear_z = PreserveSignedMagnitude(settings->joystick_scale.linear_z, limits.linear_z);
  settings->joystick_scale.angular_yaw = PreserveSignedMagnitude(settings->joystick_scale.angular_yaw, limits.angular_yaw);
  settings->keyboard_scale.linear_x = PreserveSignedMagnitude(settings->keyboard_scale.linear_x, limits.linear_x);
  settings->keyboard_scale.linear_y = PreserveSignedMagnitude(settings->keyboard_scale.linear_y, limits.linear_y);
  settings->keyboard_scale.linear_z = PreserveSignedMagnitude(settings->keyboard_scale.linear_z, limits.linear_z);
  settings->keyboard_scale.angular_yaw = PreserveSignedMagnitude(settings->keyboard_scale.angular_yaw, limits.angular_yaw);
}

}  // namespace remote_control
