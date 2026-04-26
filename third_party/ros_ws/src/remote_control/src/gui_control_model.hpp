#pragma once

#include <cstddef>
#include <deque>
#include <set>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>

namespace remote_control {

struct MotionScale {
  double linear_x = 0.6;
  double linear_y = 0.6;
  double linear_z = 0.5;
  double angular_yaw = 1.0;
};

struct ActionLogEntry {
  std::string timestamp;
  std::string source;
  std::string action_name;
  std::string service_name;
  std::string result;
};

struct JoyAxisState {
  int index = 0;
  double value = 0.0;
};

struct JoyButtonState {
  int index = 0;
  bool pressed = false;
};

struct JoyStateSnapshot {
  std::vector<JoyAxisState> axes;
  std::vector<JoyButtonState> buttons;
};

struct ActionEditorColumnWidths {
  int service_min_width = 320;
  int buttons_min_width = 180;
  int keyboard_width = 56;
  int small_button_width = 52;
};

struct JoystickStateLayoutMetrics {
  int axis_card_min_width = 120;
  int axis_card_min_height = 104;
  int button_cell_min_width = 52;
  int button_cell_min_height = 104;
};

JoyStateSnapshot BuildJoyStateSnapshot(const std::vector<double>& axes, const std::vector<int>& buttons);
std::size_t AxisGridColumnsForWidth(std::size_t axis_count, int available_width);
std::size_t ButtonGridColumnsForWidth(std::size_t button_count, int available_width);
ActionEditorColumnWidths DefaultActionEditorColumnWidths();
JoystickStateLayoutMetrics DefaultJoystickStateLayoutMetrics();

class GuiControlModel {
 public:
  explicit GuiControlModel(std::size_t max_action_history = 20);

  void PressKey(char key);
  void ReleaseKey(char key);
  void ClearKeyboard();

  bool IsKeyPressed(char key) const;
  std::string PressedKeysText() const;
  geometry_msgs::msg::Twist BuildKeyboardTwist(const MotionScale& scale) const;

  void RecordAction(std::string timestamp, std::string source, std::string action_name, std::string service_name, std::string result);
  std::vector<std::string> ActionHistoryText() const;
  std::string LatestActionStatusText() const;

 private:
  static char NormalizeKey(char key);
  static std::string KeyLabel(char key);

  std::size_t max_action_history_ = 20;
  std::set<char> pressed_keys_;
  std::deque<ActionLogEntry> action_history_;
};

}  // namespace remote_control
