#include "gui_control_model.hpp"

#include <algorithm>
#include <cctype>
#include <sstream>
#include <utility>

namespace remote_control {

JoyStateSnapshot BuildJoyStateSnapshot(const std::vector<double>& axes, const std::vector<int>& buttons) {
  JoyStateSnapshot snapshot;
  snapshot.axes.reserve(axes.size());
  for (std::size_t index = 0; index < axes.size(); ++index) {
    snapshot.axes.push_back(JoyAxisState{static_cast<int>(index), axes[index]});
  }

  snapshot.buttons.reserve(buttons.size());
  for (std::size_t index = 0; index < buttons.size(); ++index) {
    snapshot.buttons.push_back(JoyButtonState{static_cast<int>(index), buttons[index] != 0});
  }
  return snapshot;
}

std::size_t ButtonGridColumnsForCount(std::size_t button_count) {
  if (button_count == 0) {
    return 1;
  }
  if (button_count <= 8) {
    return button_count;
  }
  return (button_count + 1) / 2;
}

int AxisCardWidthForCount(std::size_t axis_count, int available_width) {
  constexpr int kMinCardWidth = 72;
  constexpr int kComfortableCardWidth = 104;
  constexpr int kCompactCardWidth = 96;
  if (axis_count == 0 || available_width <= 0) {
    return kMinCardWidth;
  }
  const int max_card_width = axis_count <= 8 ? kComfortableCardWidth : kCompactCardWidth;
  const int per_axis_width = available_width / static_cast<int>(axis_count);
  return std::clamp(per_axis_width, kMinCardWidth, max_card_width);
}

GuiControlModel::GuiControlModel(std::size_t max_action_history) : max_action_history_(max_action_history == 0 ? 1 : max_action_history) {}

void GuiControlModel::PressKey(char key) { pressed_keys_.insert(NormalizeKey(key)); }

void GuiControlModel::ReleaseKey(char key) { pressed_keys_.erase(NormalizeKey(key)); }

void GuiControlModel::ClearKeyboard() { pressed_keys_.clear(); }

bool GuiControlModel::IsKeyPressed(char key) const { return pressed_keys_.find(NormalizeKey(key)) != pressed_keys_.end(); }

std::string GuiControlModel::PressedKeysText() const {
  if (pressed_keys_.empty()) {
    return "None";
  }

  std::ostringstream out;
  bool first = true;
  for (const char key : pressed_keys_) {
    if (!first) {
      out << ' ';
    }
    first = false;
    out << KeyLabel(key);
  }
  return out.str();
}

geometry_msgs::msg::Twist GuiControlModel::BuildKeyboardTwist(const MotionScale& scale) const {
  geometry_msgs::msg::Twist command;

  const double forward = IsKeyPressed('w') ? 1.0 : 0.0;
  const double backward = IsKeyPressed('s') ? 1.0 : 0.0;
  const double left = IsKeyPressed('a') ? 1.0 : 0.0;
  const double right = IsKeyPressed('d') ? 1.0 : 0.0;
  const double up = IsKeyPressed('r') ? 1.0 : 0.0;
  const double down = IsKeyPressed('f') ? 1.0 : 0.0;
  const double yaw_left = IsKeyPressed('j') ? 1.0 : 0.0;
  const double yaw_right = IsKeyPressed('l') ? 1.0 : 0.0;

  command.linear.x = (forward - backward) * scale.linear_x;
  command.linear.y = (left - right) * scale.linear_y;
  command.linear.z = (up - down) * scale.linear_z;
  command.angular.z = (yaw_left - yaw_right) * scale.angular_yaw;
  return command;
}

void GuiControlModel::RecordAction(std::string timestamp, std::string source, std::string action_name, std::string service_name, std::string result) {
  action_history_.push_back(
      ActionLogEntry{std::move(timestamp), std::move(source), std::move(action_name), std::move(service_name), std::move(result)});
  while (action_history_.size() > max_action_history_) {
    action_history_.pop_front();
  }
}

std::vector<std::string> GuiControlModel::ActionHistoryText() const {
  std::vector<std::string> result;
  result.reserve(action_history_.size());
  for (const ActionLogEntry& entry : action_history_) {
    std::ostringstream line;
    line << entry.timestamp << " [" << entry.source << "] " << entry.action_name << " -> " << entry.service_name << " : " << entry.result;
    result.push_back(line.str());
  }
  return result;
}

char GuiControlModel::NormalizeKey(char key) {
  if (key == ' ') {
    return key;
  }
  return static_cast<char>(std::tolower(static_cast<unsigned char>(key)));
}

std::string GuiControlModel::KeyLabel(char key) {
  if (key == ' ') {
    return "Space";
  }
  return std::string(1, static_cast<char>(std::toupper(static_cast<unsigned char>(key))));
}

}  // namespace remote_control
