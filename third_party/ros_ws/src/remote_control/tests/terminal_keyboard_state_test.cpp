#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <string>

#include "remote_control_node.hpp"

namespace {

using Clock = std::chrono::steady_clock;

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

void TestMovementKeysExpire() {
  remote_control::TerminalKeyState state(std::chrono::milliseconds(500));
  const Clock::time_point start{};

  state.HandleKey('w', start);
  geometry_msgs::msg::Twist command = state.BuildTwist(start + std::chrono::milliseconds(499), 0.6, 0.6, 0.5, 1.0);
  ExpectNear(command.linear.x, 0.6, "w should move forward before key timeout");

  command = state.BuildTwist(start + std::chrono::milliseconds(501), 0.6, 0.6, 0.5, 1.0);
  ExpectNear(command.linear.x, 0.0, "w should stop after key timeout");
}

void TestSpaceClearsMovement() {
  remote_control::TerminalKeyState state(std::chrono::milliseconds(500));
  const Clock::time_point start{};

  state.HandleKey('w', start);
  state.HandleKey(' ', start + std::chrono::milliseconds(10));
  const geometry_msgs::msg::Twist command = state.BuildTwist(start + std::chrono::milliseconds(20), 0.6, 0.6, 0.5, 1.0);
  ExpectNear(command.linear.x, 0.0, "space should clear active movement keys");
}

void TestActionKeysQueueEvents() {
  remote_control::TerminalKeyState state(std::chrono::milliseconds(500));
  state.RegisterEventKey('t', "takeoff");

  state.HandleKey('T', Clock::time_point{});
  const std::optional<std::string> action = state.ConsumeEvent();
  Expect(action.has_value(), "registered action key should queue an event");
  Expect(*action == "takeoff", "queued action should match registered action name");
  Expect(!state.ConsumeEvent().has_value(), "event queue should be empty after consuming event");
}

}  // namespace

int main() {
  TestMovementKeysExpire();
  TestSpaceClearsMovement();
  TestActionKeysQueueEvents();
  return 0;
}
