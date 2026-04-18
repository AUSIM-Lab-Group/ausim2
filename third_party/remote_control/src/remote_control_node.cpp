#include "remote_control_node.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <termios.h>

#include <sys/select.h>
#include <unistd.h>

namespace remote_control {
namespace {

constexpr char kDefaultJoyTopic[] = "/joy";
constexpr char kDefaultCmdVelTopic[] = "/joy/cmd_vel";

double ApplyDeadzone(double value, double deadzone) {
  if (std::abs(value) < deadzone) {
    return 0.0;
  }
  return value;
}

char NormalizeKey(char key) { return static_cast<char>(std::tolower(static_cast<unsigned char>(key))); }

}  // namespace

TerminalKeyboard::TerminalKeyboard(bool enabled, double key_timeout_seconds)
    : key_timeout_(std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(key_timeout_seconds))),
      original_termios_(new termios{}) {
  if (!enabled) {
    return;
  }
  if (!isatty(STDIN_FILENO)) {
    return;
  }
  if (tcgetattr(STDIN_FILENO, original_termios_) != 0) {
    return;
  }

  termios raw = *original_termios_;
  raw.c_lflag &= static_cast<unsigned int>(~(ICANON | ECHO));
  raw.c_cc[VMIN] = 0;
  raw.c_cc[VTIME] = 0;
  if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
    return;
  }

  had_termios_ = true;
  terminal_configured_ = true;
  available_.store(true);
  running_.store(true);
  reader_thread_ = std::thread(&TerminalKeyboard::ReaderLoop, this);
}

TerminalKeyboard::~TerminalKeyboard() {
  running_.store(false);
  if (reader_thread_.joinable()) {
    reader_thread_.join();
  }
  if (terminal_configured_ && had_termios_ && original_termios_ != nullptr) {
    tcsetattr(STDIN_FILENO, TCSANOW, original_termios_);
  }
  delete original_termios_;
}

void TerminalKeyboard::RegisterEventKey(char key, std::string event_name) {
  if (key == '\0') {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  event_keys_[NormalizeKey(key)] = std::move(event_name);
}

void TerminalKeyboard::ClearEventKeys() {
  std::lock_guard<std::mutex> lock(mutex_);
  event_keys_.clear();
}

geometry_msgs::msg::Twist TerminalKeyboard::BuildTwist(double linear_x_scale, double linear_y_scale, double linear_z_scale,
                                                       double angular_yaw_scale) const {
  geometry_msgs::msg::Twist command;
  const TimePoint now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(mutex_);

  const double forward = key_state_.forward_until > now ? 1.0 : 0.0;
  const double backward = key_state_.backward_until > now ? 1.0 : 0.0;
  const double left = key_state_.left_until > now ? 1.0 : 0.0;
  const double right = key_state_.right_until > now ? 1.0 : 0.0;
  const double up = key_state_.up_until > now ? 1.0 : 0.0;
  const double down = key_state_.down_until > now ? 1.0 : 0.0;
  const double yaw_left = key_state_.yaw_left_until > now ? 1.0 : 0.0;
  const double yaw_right = key_state_.yaw_right_until > now ? 1.0 : 0.0;

  command.linear.x = (forward - backward) * linear_x_scale;
  command.linear.y = (left - right) * linear_y_scale;
  command.linear.z = (up - down) * linear_z_scale;
  command.angular.z = (yaw_left - yaw_right) * angular_yaw_scale;
  return command;
}

std::optional<std::string> TerminalKeyboard::ConsumeEvent() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (pending_events_.empty()) {
    return std::nullopt;
  }
  std::string event_name = std::move(pending_events_.front());
  pending_events_.pop_front();
  return event_name;
}

void TerminalKeyboard::ReaderLoop() {
  while (running_.load()) {
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(STDIN_FILENO, &read_fds);

    timeval timeout{};
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000;
    const int ready = select(STDIN_FILENO + 1, &read_fds, nullptr, nullptr, &timeout);
    if (ready <= 0 || !FD_ISSET(STDIN_FILENO, &read_fds)) {
      continue;
    }

    char value = '\0';
    const ssize_t bytes = read(STDIN_FILENO, &value, 1);
    if (bytes == 1) {
      HandleChar(value);
    }
  }
}

void TerminalKeyboard::HandleChar(char value) {
  const TimePoint until = std::chrono::steady_clock::now() + key_timeout_;
  std::lock_guard<std::mutex> lock(mutex_);
  switch (NormalizeKey(value)) {
    case 'w':
      key_state_.forward_until = until;
      return;
    case 's':
      key_state_.backward_until = until;
      return;
    case 'a':
      key_state_.left_until = until;
      return;
    case 'd':
      key_state_.right_until = until;
      return;
    case 'r':
      key_state_.up_until = until;
      return;
    case 'f':
      key_state_.down_until = until;
      return;
    case 'j':
      key_state_.yaw_left_until = until;
      return;
    case 'l':
      key_state_.yaw_right_until = until;
      return;
    case ' ':
      ClearMovementLocked();
      return;
    default:
      break;
  }

  // Non-movement keys: look up the user-configured event map.
  const auto it = event_keys_.find(NormalizeKey(value));
  if (it != event_keys_.end()) {
    pending_events_.push_back(it->second);
  }
}

void TerminalKeyboard::ClearMovementLocked() {
  const TimePoint zero{};
  key_state_.forward_until = zero;
  key_state_.backward_until = zero;
  key_state_.left_until = zero;
  key_state_.right_until = zero;
  key_state_.up_until = zero;
  key_state_.down_until = zero;
  key_state_.yaw_left_until = zero;
  key_state_.yaw_right_until = zero;
}

RemoteControlNode::RemoteControlNode() : Node("remote_control_node") {
  LoadParameters();
  LoadActionBindings();
  cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  joy_subscription_ = create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_, rclcpp::SensorDataQoS(), std::bind(&RemoteControlNode::OnJoy, this, std::placeholders::_1));

  keyboard_ = std::make_unique<TerminalKeyboard>(keyboard_enabled_, keyboard_key_timeout_seconds_);
  if (keyboard_enabled_ && !keyboard_->available()) {
    RCLCPP_WARN(get_logger(), "keyboard fallback requested but stdin is not a TTY; keyboard control is disabled");
  }
  for (const auto& [action_name, binding] : action_bindings_) {
    if (binding.keyboard_key != '\0') {
      keyboard_->RegisterEventKey(binding.keyboard_key, action_name);
    }
    action_clients_.emplace(action_name, create_client<std_srvs::srv::Trigger>(binding.service_name));
  }

  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / publish_rate_hz_));
  publish_timer_ = create_wall_timer(period, std::bind(&RemoteControlNode::OnPublishTimer, this));
}

void RemoteControlNode::LoadParameters() {
  joy_topic_ = declare_parameter<std::string>("topics.joy", kDefaultJoyTopic);
  cmd_vel_topic_ = declare_parameter<std::string>("topics.cmd_vel", kDefaultCmdVelTopic);

  publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 30.0);
  joy_timeout_seconds_ = declare_parameter<double>("joy_timeout", 0.5);
  deadzone_ = declare_parameter<double>("deadzone", 0.15);
  require_enable_button_ = declare_parameter<bool>("require_enable_button", false);
  enable_button_ = declare_parameter<int>("enable_button", 4);
  axis_linear_x_ = declare_parameter<int>("axes.linear.x", 4);
  axis_linear_y_ = declare_parameter<int>("axes.linear.y", 0);
  axis_linear_z_ = declare_parameter<int>("axes.linear.z", 1);
  axis_angular_yaw_ = declare_parameter<int>("axes.angular.yaw", 3);
  scale_linear_x_ = declare_parameter<double>("scale.linear.x", 0.6);
  scale_linear_y_ = declare_parameter<double>("scale.linear.y", 0.6);
  scale_linear_z_ = declare_parameter<double>("scale.linear.z", 0.5);
  scale_angular_yaw_ = declare_parameter<double>("scale.angular.yaw", 1.0);

  command_cooldown_seconds_ = declare_parameter<double>("command_cooldown", 0.5);
  keyboard_enabled_ = declare_parameter<bool>("keyboard.enabled", true);
  keyboard_key_timeout_seconds_ = declare_parameter<double>("keyboard.key_timeout", 0.25);
  keyboard_scale_linear_x_ = declare_parameter<double>("keyboard.scale.linear.x", 0.6);
  keyboard_scale_linear_y_ = declare_parameter<double>("keyboard.scale.linear.y", 0.6);
  keyboard_scale_linear_z_ = declare_parameter<double>("keyboard.scale.linear.z", 0.5);
  keyboard_scale_angular_yaw_ = declare_parameter<double>("keyboard.scale.angular.yaw", 1.0);
  motion_suppress_duration_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(declare_parameter<double>("motion_suppress_after_command", 0.3)));

  if (publish_rate_hz_ <= 0.0) {
    throw std::runtime_error("publish_rate_hz must be positive");
  }
  if (joy_timeout_seconds_ < 0.0) {
    throw std::runtime_error("joy_timeout must be non-negative");
  }
}

void RemoteControlNode::LoadActionBindings() {
  struct ActionDefault {
    const char* name;
    const char* service_name;
    std::vector<std::int64_t> buttons;
    char keyboard_key;
  };
  const ActionDefault action_defaults[] = {
      {"action1", "/joy/action1", {4, 0}, 't'},
      {"action2", "/joy/action2", {5, 0}, 'g'},
      {"action3", "/joy/action3", {6, 7}, 'x'},
      {"action4", "/joy/action4", {-1}, 'q'},
      {"action5", "", {-1}, '\0'},
      {"action6", "", {-1}, '\0'},
      {"action7", "", {-1}, '\0'},
      {"action8", "", {-1}, '\0'},
  };

  for (const ActionDefault& def : action_defaults) {
    const std::string name(def.name);
    const std::string service_param = "actions." + name + ".service";
    const std::string buttons_param = "actions." + name + ".buttons";
    const std::string keyboard_param = "actions." + name + ".keyboard";
    const std::string service_name = declare_parameter<std::string>(service_param, def.service_name);
    std::vector<std::int64_t> buttons = declare_parameter<std::vector<std::int64_t>>(buttons_param, def.buttons);
    std::string keyboard_str =
        declare_parameter<std::string>(keyboard_param, def.keyboard_key == '\0' ? "" : std::string(1, def.keyboard_key));

    ActionBinding binding;
    binding.service_name = service_name;
    binding.buttons = ToIntVector(buttons);
    // `-1` remains the config sentinel for "no joystick binding".
    binding.buttons.erase(std::remove(binding.buttons.begin(), binding.buttons.end(), -1), binding.buttons.end());
    binding.keyboard_key = keyboard_str.empty() ? '\0' : keyboard_str.front();
    if (binding.service_name.empty()) {
      continue;
    }
    action_bindings_.emplace(name, std::move(binding));
    action_combo_active_.emplace(name, false);
  }
}

void RemoteControlNode::OnJoy(const sensor_msgs::msg::Joy::SharedPtr message) {
  if (!have_joy_message_) {
    RCLCPP_INFO(get_logger(), "received first joy message on %s", joy_topic_.c_str());
  }
  latest_joy_message_ = *message;
  have_joy_message_ = true;
  last_joy_message_time_ = now();

  for (auto& [action_name, binding] : action_bindings_) {
    const bool pressed = !binding.buttons.empty() && ButtonsPressed(*message, binding.buttons);
    bool& was_active = action_combo_active_[action_name];
    if (pressed && !was_active) {
      TriggerAction(action_name, "joystick");
    }
    was_active = pressed;
  }
}

void RemoteControlNode::OnPublishTimer() {
  while (keyboard_ != nullptr) {
    std::optional<std::string> action = keyboard_->ConsumeEvent();
    if (!action.has_value()) {
      break;
    }
    TriggerAction(*action, "keyboard");
  }

  geometry_msgs::msg::Twist command;
  const bool joystick_active = have_joy_message_ && (now() - last_joy_message_time_).seconds() <= joy_timeout_seconds_;
  if (!joystick_active) {
    for (auto& [name, active] : action_combo_active_) {
      (void)name;
      active = false;
    }
  }

  if (std::chrono::steady_clock::now() < suppress_motion_until_) {
    command = geometry_msgs::msg::Twist();
    UpdateInputMode(joystick_active ? InputMode::kJoystick : (keyboard_ != nullptr && keyboard_->available() ? InputMode::kKeyboard : InputMode::kNone));
  } else if (joystick_active) {
    command = BuildJoyTwist(latest_joy_message_);
    UpdateInputMode(InputMode::kJoystick);
  } else if (keyboard_ != nullptr && keyboard_->available()) {
    command = keyboard_->BuildTwist(keyboard_scale_linear_x_, keyboard_scale_linear_y_, keyboard_scale_linear_z_, keyboard_scale_angular_yaw_);
    UpdateInputMode(InputMode::kKeyboard);
  } else {
    UpdateInputMode(InputMode::kNone);
  }

  cmd_vel_publisher_->publish(command);
}

void RemoteControlNode::TriggerAction(const std::string& action_name, const char* source) {
  if (action_name.empty()) {
    return;
  }
  const auto binding_it = action_bindings_.find(action_name);
  const auto client_it = action_clients_.find(action_name);
  if (binding_it == action_bindings_.end() || client_it == action_clients_.end() || !client_it->second) {
    RCLCPP_WARN(get_logger(), "ignoring unconfigured action slot '%s'", action_name.c_str());
    return;
  }

  const auto now_steady = std::chrono::steady_clock::now();
  if (last_discrete_trigger_time_ != std::chrono::steady_clock::time_point{} &&
      std::chrono::duration<double>(now_steady - last_discrete_trigger_time_).count() < command_cooldown_seconds_) {
    return;
  }

  last_discrete_trigger_time_ = now_steady;
  suppress_motion_until_ = now_steady + motion_suppress_duration_;
  PublishZeroCommand();

  const std::string service_name = binding_it->second.service_name;
  auto client = client_it->second;
  RCLCPP_INFO(get_logger(), "triggering %s -> %s from %s", action_name.c_str(), service_name.c_str(), source);

  if (!client->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "service %s is not ready for action slot %s", service_name.c_str(), action_name.c_str());
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  client->async_send_request(
      request, [this, action_name, service_name](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        try {
          const auto response = future.get();
          if (!response) {
            RCLCPP_WARN(get_logger(), "action slot %s (%s) returned no response", action_name.c_str(), service_name.c_str());
            return;
          }
          if (response->success) {
            RCLCPP_INFO(get_logger(), "action slot %s (%s) succeeded: %s", action_name.c_str(), service_name.c_str(),
                        response->message.c_str());
            return;
          }
          RCLCPP_WARN(get_logger(), "action slot %s (%s) failed: %s", action_name.c_str(), service_name.c_str(),
                      response->message.c_str());
        } catch (const std::exception& error) {
          RCLCPP_WARN(get_logger(), "action slot %s (%s) call threw: %s", action_name.c_str(), service_name.c_str(), error.what());
        }
      });
}

void RemoteControlNode::PublishZeroCommand() { cmd_vel_publisher_->publish(geometry_msgs::msg::Twist()); }

void RemoteControlNode::UpdateInputMode(InputMode mode) {
  if (mode == input_mode_) {
    return;
  }

  input_mode_ = mode;
  switch (input_mode_) {
    case InputMode::kJoystick:
      RCLCPP_INFO(get_logger(), "using joystick control");
      break;
    case InputMode::kKeyboard:
      RCLCPP_INFO(get_logger(),
                  "no fresh joystick input on %s, switching to keyboard fallback; ensure joy_node is running and autorepeat_rate is enabled",
                  joy_topic_.c_str());
      break;
    case InputMode::kNone:
      RCLCPP_INFO(get_logger(),
                  "no fresh joystick input on %s and no keyboard TTY, publishing zero cmd_vel; ensure joy_node is running and autorepeat_rate is enabled",
                  joy_topic_.c_str());
      break;
  }
}

geometry_msgs::msg::Twist RemoteControlNode::BuildJoyTwist(const sensor_msgs::msg::Joy& message) const {
  geometry_msgs::msg::Twist command;
  if (require_enable_button_ && !ButtonPressed(message, enable_button_)) {
    return command;
  }

  command.linear.x = AxisValue(message, axis_linear_x_, scale_linear_x_);
  command.linear.y = AxisValue(message, axis_linear_y_, scale_linear_y_);
  command.linear.z = AxisValue(message, axis_linear_z_, scale_linear_z_);
  command.angular.z = AxisValue(message, axis_angular_yaw_, scale_angular_yaw_);
  return command;
}

bool RemoteControlNode::ButtonPressed(const sensor_msgs::msg::Joy& message, int button_index) const {
  return button_index >= 0 && button_index < static_cast<int>(message.buttons.size()) && message.buttons[button_index] != 0;
}

bool RemoteControlNode::ButtonsPressed(const sensor_msgs::msg::Joy& message, const std::vector<int>& button_indices) const {
  if (button_indices.empty()) {
    return false;
  }
  return std::all_of(button_indices.begin(), button_indices.end(), [this, &message](int button_index) {
    return ButtonPressed(message, button_index);
  });
}

double RemoteControlNode::AxisValue(const sensor_msgs::msg::Joy& message, int axis_index, double scale) const {
  if (axis_index < 0 || axis_index >= static_cast<int>(message.axes.size())) {
    return 0.0;
  }
  return ApplyDeadzone(message.axes[axis_index], deadzone_) * scale;
}

std::vector<int> RemoteControlNode::ToIntVector(const std::vector<std::int64_t>& values) {
  std::vector<int> result;
  result.reserve(values.size());
  for (const std::int64_t value : values) {
    result.push_back(static_cast<int>(value));
  }
  return result;
}

}  // namespace remote_control
