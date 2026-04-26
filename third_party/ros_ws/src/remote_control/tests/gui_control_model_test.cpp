#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "gui_control_model.hpp"

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

remote_control::MotionScale BuildScale() {
  remote_control::MotionScale scale;
  scale.linear_x = 0.6;
  scale.linear_y = 0.7;
  scale.linear_z = 0.8;
  scale.angular_yaw = 0.9;
  return scale;
}

void TestKeyboardTracksMultiplePressedKeys() {
  remote_control::GuiControlModel model;
  model.PressKey('w');
  model.PressKey('a');

  const geometry_msgs::msg::Twist command = model.BuildKeyboardTwist(BuildScale());
  ExpectNear(command.linear.x, 0.6, "W should contribute forward velocity");
  ExpectNear(command.linear.y, 0.7, "A should contribute left velocity while W remains held");
  Expect(model.PressedKeysText() == "A W", "pressed key display should include both keys sorted");
}

void TestReleaseOnlyClearsReleasedKey() {
  remote_control::GuiControlModel model;
  model.PressKey('w');
  model.PressKey('a');
  model.ReleaseKey('a');

  const geometry_msgs::msg::Twist command = model.BuildKeyboardTwist(BuildScale());
  ExpectNear(command.linear.x, 0.6, "W should remain active after releasing A");
  ExpectNear(command.linear.y, 0.0, "A contribution should stop after releasing A");
  Expect(model.PressedKeysText() == "W", "pressed key display should only include W");
}

void TestFocusLossClearsKeyboardState() {
  remote_control::GuiControlModel model;
  model.PressKey('w');
  model.PressKey('j');
  model.ClearKeyboard();

  const geometry_msgs::msg::Twist command = model.BuildKeyboardTwist(BuildScale());
  ExpectNear(command.linear.x, 0.0, "focus loss should clear forward velocity");
  ExpectNear(command.angular.z, 0.0, "focus loss should clear yaw velocity");
  Expect(model.PressedKeysText() == "None", "pressed key display should show none after focus loss");
}

void TestActionHistoryKeepsNewestEntries() {
  remote_control::GuiControlModel model(2);
  model.RecordAction("12:00:00", "keyboard", "action1", "/joy/action1", "success");
  model.RecordAction("12:00:01", "joystick", "action2", "/joy/action2", "not ready");
  model.RecordAction("12:00:02", "keyboard", "action3", "/joy/action3", "failed");

  const std::vector<std::string> history = model.ActionHistoryText();
  Expect(history.size() == 2, "history should keep configured number of newest entries");
  Expect(history[0].find("action2") != std::string::npos, "oldest retained entry should be action2");
  Expect(history[1].find("action3") != std::string::npos, "newest retained entry should be action3");
}

void TestJoySnapshotIncludesEveryAxisAndButton() {
  std::vector<double> axes = {-1.0, -0.25, 0.0, 0.25, 1.0};
  std::vector<int> buttons = {0, 1, 0, 1};

  const remote_control::JoyStateSnapshot snapshot = remote_control::BuildJoyStateSnapshot(axes, buttons);

  Expect(snapshot.axes.size() == axes.size(), "joy snapshot should include every axis");
  Expect(snapshot.buttons.size() == buttons.size(), "joy snapshot should include every button");
  Expect(snapshot.axes[0].index == 0, "first axis index should be preserved");
  ExpectNear(snapshot.axes[0].value, -1.0, "first axis value should be preserved");
  Expect(snapshot.axes[2].index == 2, "zero axis index should be preserved");
  ExpectNear(snapshot.axes[2].value, 0.0, "zero axis value should still be present");
  Expect(snapshot.buttons[0].index == 0, "first button index should be preserved");
  Expect(!snapshot.buttons[0].pressed, "button 0 should be released");
  Expect(snapshot.buttons[1].pressed, "button 1 should be pressed");
}

void TestJoySnapshotFollowsLatestMessageShape() {
  const remote_control::JoyStateSnapshot small = remote_control::BuildJoyStateSnapshot({0.5}, {1});
  const remote_control::JoyStateSnapshot large = remote_control::BuildJoyStateSnapshot({0.1, 0.2, 0.3}, {0, 0, 1, 0});

  Expect(small.axes.size() == 1, "small snapshot should expose one axis");
  Expect(small.buttons.size() == 1, "small snapshot should expose one button");
  Expect(large.axes.size() == 3, "large snapshot should expose latest axis count");
  Expect(large.buttons.size() == 4, "large snapshot should expose latest button count");
  Expect(large.buttons[2].pressed, "latest button state should be preserved");
}

void TestButtonGridUsesAtMostTwoRows() {
  Expect(remote_control::ButtonGridColumnsForCount(0) == 1, "empty button grid should still have one column");
  Expect(remote_control::ButtonGridColumnsForCount(1) == 1, "one button should use one column");
  Expect(remote_control::ButtonGridColumnsForCount(4) == 4, "four buttons should fit on one row");
  Expect(remote_control::ButtonGridColumnsForCount(8) == 8, "eight buttons should fit on one row");
  Expect(remote_control::ButtonGridColumnsForCount(9) == 5, "nine buttons should wrap to two compact rows");
  Expect(remote_control::ButtonGridColumnsForCount(16) == 8, "sixteen buttons should use two rows of eight");
}

void TestAxisCardWidthFitsCommonJoystickCounts() {
  Expect(remote_control::AxisCardWidthForCount(0, 1040) == 72, "empty axis card width should use compact minimum");
  Expect(remote_control::AxisCardWidthForCount(6, 1040) == 104, "six axes should use comfortable card width");
  Expect(remote_control::AxisCardWidthForCount(8, 1040) == 104, "eight axes should use comfortable card width");
  Expect(remote_control::AxisCardWidthForCount(10, 1040) <= 96, "ten axes should shrink enough to fit one row");
  Expect(remote_control::AxisCardWidthForCount(12, 1040) >= 72, "many axes should not shrink below readable width");
}

}  // namespace

int main() {
  TestKeyboardTracksMultiplePressedKeys();
  TestReleaseOnlyClearsReleasedKey();
  TestFocusLossClearsKeyboardState();
  TestActionHistoryKeepsNewestEntries();
  TestJoySnapshotIncludesEveryAxisAndButton();
  TestJoySnapshotFollowsLatestMessageShape();
  TestButtonGridUsesAtMostTwoRows();
  TestAxisCardWidthFitsCommonJoystickCounts();
  return 0;
}
